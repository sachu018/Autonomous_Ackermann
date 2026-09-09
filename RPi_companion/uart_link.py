#!/usr/bin/env python3
# uart_link.py — RPi-side driver for the STM32 UART link (Phase 5).
#
# Counterpart to Rover_closed_loop/Core/Src/rpi_link.c on the STM32 side.
# Physical link: RPi5 GPIO14/Pin8 (TXD) <-> STM32 PA3 (USART2_RX),
#                RPi5 GPIO15/Pin10 (RXD) <-> STM32 PA2 (USART2_TX), GND common.
# Device node: /dev/serial0 (symlinks to whatever ttyAMA the Pi5 assigns —
# was ttyAMA10 on this specific Pi, see scratchpad.md; always go through the
# alias, never hardcode the ttyAMA number).
#
# Frame formats — MUST stay byte-for-byte identical to rpi_link.c's comment
# header, since both sides hand-encode/decode independently with no shared
# schema file:
#
#   Command frame, RPi -> STM32, 8 bytes, little-endian:
#       [0]     0xAA                    header
#       [1]     0x55                    header
#       [2..3]  int16  steer_target     degrees x100  (-4500..+4500)
#       [4..5]  int16  speed_target     mm/s, signed
#       [6]     uint8  seq              rolling counter
#       [7]     uint8  checksum         XOR of bytes [0..6]
#
#   Feedback frame, STM32 -> RPi, 12 bytes, little-endian:
#       [0]     0xBB                    header
#       [1]     0x66                    header
#       [2..3]  int16  angle_L          degrees x100
#       [4..5]  int16  angle_R          degrees x100
#       [6..7]  int16  rpm_L            RPM x10
#       [8..9]  int16  rpm_R            RPM x10
#       [10]    uint8  status           bitfield, see STATUS_* below
#       [11]    uint8  checksum         XOR of bytes [0..10]
#
# The RPi only ever RECEIVES feedback frames — TX and RX are separate wires
# on a point-to-point UART, so this process never sees its own command
# frames looped back.

import struct
import threading
import time

try:
    import serial
    _PYSERIAL_AVAILABLE = True
except ImportError:
    _PYSERIAL_AVAILABLE = False

DEFAULT_PORT = "/dev/serial0"
DEFAULT_BAUD = 115200

CMD_FRAME_LEN = 8
CMD_HEADER = b"\xAA\x55"

FEEDBACK_FRAME_LEN = 12
FEEDBACK_HEADER = b"\xBB\x66"

# Mirrors AUTO_UART_TIMEOUT_US (300 ms) on the STM32 side — that's how long
# IT waits for OUR frames before declaring the link down; this is how long
# WE wait for ITS feedback frames before declaring the same thing. Slightly
# looser (500 ms) since this side has more jitter (Python + USB-less UART,
# no hard real-time guarantee) and this number is diagnostic, not a
# safety-critical failsafe like the STM32's copy is.
LINK_TIMEOUT_S = 0.5

# Status bitfield — must match RPI_STATUS_* in rpi_link.h
STATUS_ARMED       = 1 << 0
STATUS_STEER_FAULT = 1 << 1
STATUS_AUTO_ACTIVE = 1 << 2
STATUS_RC_OK       = 1 << 3


class FeedbackData:
    __slots__ = (
        "angle_L_deg", "angle_R_deg", "rpm_L", "rpm_R",
        "armed", "steer_fault", "auto_active", "rc_ok",
        "valid", "age_s",
    )

    def __init__(self):
        self.angle_L_deg = 0.0
        self.angle_R_deg = 0.0
        self.rpm_L = 0.0
        self.rpm_R = 0.0
        self.armed = False
        self.steer_fault = False
        self.auto_active = False
        self.rc_ok = False
        self.valid = False
        self.age_s = float("inf")


def _checksum(b: bytes) -> int:
    x = 0
    for byte in b:
        x ^= byte
    return x


class STM32Link:
    """Owns the serial port + a background reader thread that continuously
    parses feedback frames out of the byte stream (no hardware IDLE-line
    equivalent on this side, so framing is done by scanning for the header
    bytes rather than relying on inter-frame gaps)."""

    def __init__(self, port: str = DEFAULT_PORT, baud: int = DEFAULT_BAUD):
        self._latest = FeedbackData()
        self._lock = threading.Lock()
        self._last_good_ts = 0.0
        self._frames_ok = 0
        self._frames_bad = 0  # header found but checksum failed
        self._seq = 0
        self._running = False
        self._ser = None
        self._thread = None

        if not _PYSERIAL_AVAILABLE:
            print("[UART] pyserial not available on this host — stub mode.")
            return

        try:
            self._ser = serial.Serial(port, baud, timeout=0.05)
            print(f"[UART] Opened {port} @ {baud} baud.")
        except Exception as e:
            print(f"[UART] Failed to open {port}: {e}")
            self._ser = None
            return

        self._running = True
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

    def is_open(self) -> bool:
        return self._ser is not None

    # ── RX: background frame parser ────────────────────────────────────
    def _reader_loop(self):
        buf = bytearray()
        while self._running:
            try:
                chunk = self._ser.read(64)
            except Exception as e:
                print(f"[UART] Read error: {e}")
                time.sleep(0.5)
                continue

            if not chunk:
                continue
            buf.extend(chunk)

            # Scan for a complete, valid feedback frame anywhere in the
            # buffer, resyncing byte-by-byte on failure (no IDLE-line gap
            # to rely on here, unlike the STM32 firmware side).
            while True:
                idx = buf.find(FEEDBACK_HEADER)
                if idx < 0:
                    # No header at all — keep only the last byte in case it's
                    # the first half of a header split across reads.
                    if len(buf) > 1:
                        del buf[:-1]
                    break

                if idx > 0:
                    del buf[:idx]  # drop garbage before the header

                if len(buf) < FEEDBACK_FRAME_LEN:
                    break  # wait for more bytes

                frame = bytes(buf[:FEEDBACK_FRAME_LEN])
                if _checksum(frame[:-1]) == frame[-1]:
                    self._handle_frame(frame)
                    del buf[:FEEDBACK_FRAME_LEN]
                else:
                    # Checksum mismatch — this was probably a false-positive
                    # header match inside other data. Drop just the header
                    # bytes and keep scanning rather than the whole frame.
                    with self._lock:
                        self._frames_bad += 1
                    del buf[:2]

    def _handle_frame(self, frame: bytes):
        # "<xxhhhhBB": skip the 2 header bytes, then angle_L, angle_R,
        # rpm_L, rpm_R (int16 each), status (uint8), checksum (uint8).
        angle_L, angle_R, rpm_L, rpm_R, status, _ = struct.unpack("<xxhhhhBB", frame)
        with self._lock:
            self._latest.angle_L_deg = angle_L / 100.0
            self._latest.angle_R_deg = angle_R / 100.0
            self._latest.rpm_L = rpm_L / 10.0
            self._latest.rpm_R = rpm_R / 10.0
            self._latest.armed       = bool(status & STATUS_ARMED)
            self._latest.steer_fault = bool(status & STATUS_STEER_FAULT)
            self._latest.auto_active = bool(status & STATUS_AUTO_ACTIVE)
            self._latest.rc_ok       = bool(status & STATUS_RC_OK)
            self._latest.valid = True
            self._last_good_ts = time.monotonic()
            self._frames_ok += 1

    def read(self) -> FeedbackData:
        """Latest feedback sample. valid=False if nothing arrived within
        LINK_TIMEOUT_S — call this at whatever rate your script needs."""
        with self._lock:
            age = time.monotonic() - self._last_good_ts if self._last_good_ts else float("inf")
            self._latest.age_s = age
            if age > LINK_TIMEOUT_S:
                self._latest.valid = False
            # Return a shallow copy so the caller can't mutate our state
            d = FeedbackData()
            for slot in FeedbackData.__slots__:
                setattr(d, slot, getattr(self._latest, slot))
            return d

    def stats(self):
        """(frames_ok, frames_bad) counters since open() — for connectivity diagnostics."""
        with self._lock:
            return self._frames_ok, self._frames_bad

    # ── TX: command frames ──────────────────────────────────────────────
    def send_command(self, steer_target_deg: float, speed_target_ms: float):
        """Sends one command frame. Safe to call even while the STM32's SWB
        is in MANUAL — the STM32 only acts on these when SWB=AUTO, and even
        then steer=0/speed=0 just holds the actuator at center and brakes."""
        if self._ser is None:
            return

        steer_i16 = max(-4500, min(4500, int(round(steer_target_deg * 100))))
        speed_i16 = max(-32768, min(32767, int(round(speed_target_ms * 1000))))

        frame = bytearray(CMD_HEADER)
        frame += struct.pack("<hhB", steer_i16, speed_i16, self._seq & 0xFF)
        frame.append(_checksum(bytes(frame)))
        self._seq += 1

        try:
            self._ser.write(frame)
        except Exception as e:
            print(f"[UART] Write error: {e}")

    def close(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._ser is not None:
            self._ser.close()
