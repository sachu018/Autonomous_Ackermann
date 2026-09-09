# flysky_receiver.py
# Interfaces with the FlySky FS-iA6B receiver via iBUS protocol over UART5.

import time
import struct
import serial

from config import (
    IBUS_PORT, IBUS_BAUD, IBUS_CH_X, IBUS_CH_Y,
    IBUS_CH_SWC, IBUS_CH_SWD, IBUS_CH_SWB,
    DEADBAND, SWB_DEFAULT_SPEED, RC_TIMEOUT_S
)

# iBUS frame properties
IBUS_FRAME_LEN = 32
IBUS_HEADER_0 = 0x20
IBUS_HEADER_1 = 0x40
IBUS_NUM_CH = 10
IBUS_MID = 1500
IBUS_MIN = 1000
IBUS_MAX = 2000

# Switch Thresholds
SWC_THRESH_LO = 1250
SWC_THRESH_HI = 1750
SWC_SPEED_LOW = 0.30
SWC_SPEED_MID = 0.60
SWC_SPEED_HIGH = 1.00

SWB_THRESH = 1500
SWB_SPEED_LOW = 0.60
SWB_SPEED_HIGH = 1.00

SWD_THRESH = 1450


class FlySkyReceiver:
    def __init__(self):
        try:
            self.ser = serial.Serial(IBUS_PORT, IBUS_BAUD, timeout=0.05)
            print(f"[RC] Opened serial port {IBUS_PORT} at {IBUS_BAUD} baud.")
        except Exception as e:
            self.ser = None
            print(f"[RC] Warning: Failed to open serial port {IBUS_PORT}: {e}. Running in simulation.")
            
        self.channels = [IBUS_MID] * IBUS_NUM_CH
        self._last_good_time = time.monotonic()
        self._signal_ok = False

    def _read_frame(self) -> bool:
        if self.ser is None:
            return False
            
        try:
            # Look for ibus frame headers
            b0 = self.ser.read(1)
            if not b0 or b0[0] != IBUS_HEADER_0:
                return False
                
            b1 = self.ser.read(1)
            if not b1 or b1[0] != IBUS_HEADER_1:
                return False
                
            # Read remainder of the 32-byte frame
            rest = self.ser.read(IBUS_FRAME_LEN - 2)
            if len(rest) < (IBUS_FRAME_LEN - 2):
                return False
                
            frame = bytes([IBUS_HEADER_0, IBUS_HEADER_1]) + rest
            
            # Verify checksum
            calc_csum = 0xFFFF
            for i in range(30):
                calc_csum -= frame[i]
            calc_csum &= 0xFFFF
            
            rx_csum = struct.unpack_from('<H', frame, 30)[0]
            if calc_csum != rx_csum:
                return False
                
            # Unpack 10 channels (2 bytes per channel, little-endian)
            for ch in range(IBUS_NUM_CH):
                idx = 2 + ch * 2
                val = struct.unpack_from('<H', frame, idx)[0]
                self.channels[ch] = max(IBUS_MIN, min(IBUS_MAX, val))
                
            return True
        except Exception:
            return False

    @staticmethod
    def _to_float(raw: int) -> float:
        """Converts raw channel value [1000, 2000] to float [-1.0, 1.0] with a center deadband."""
        raw = max(IBUS_MIN, min(raw, IBUS_MAX))
        if raw < IBUS_MID:
            norm = (raw - IBUS_MID) / float(IBUS_MID - IBUS_MIN)
        else:
            norm = (raw - IBUS_MID) / float(IBUS_MAX - IBUS_MID)
        if abs(norm) <= DEADBAND:
            return 0.0
        return norm

    def read(self):
        """
        Reads the latest RC channels.
        Returns:
            Xn (float): Normalized steering (-1.0=Left, 1.0=Right)
            Yn (float): Normalized throttle (-1.0=Reverse, 1.0=Forward)
            swd_on (bool): Contactor arming switch state (CH6)
            mode (str): Switch SWC mode: "MANUAL", "AUTO_SLOW", "AUTO_FAST" (CH5)
            rc_ok (bool): True if receiver is connected and receiving frames
        """
        now = time.monotonic()
        
        # Flush serial input buffer if accumulating delay
        if self.ser is not None:
            try:
                if self.ser.in_waiting > 64:
                    self.ser.reset_input_buffer()
            except Exception:
                pass

        frame_ok = self._read_frame()

        if frame_ok:
            self._last_good_time = now
            self._signal_ok = True
        else:
            if self._signal_ok and (now - self._last_good_time) > RC_TIMEOUT_S:
                self._signal_ok = False
                self.channels = [IBUS_MID] * IBUS_NUM_CH

        # Extract normalized sticks
        Xn = self._to_float(self.channels[IBUS_CH_X])
        Yn = self._to_float(self.channels[IBUS_CH_Y])

        # CH6 (SWD) control: Contactor Energize Switch
        swd_on = self.channels[IBUS_CH_SWD] > SWD_THRESH

        # CH5 (SWC) control: Mode selection & forward speed limit
        raw_swc = self.channels[IBUS_CH_SWC]
        if raw_swc < SWC_THRESH_LO:
            mode = "MANUAL"
            fwd_limit = SWC_SPEED_LOW
        elif raw_swc < SWC_THRESH_HI:
            mode = "AUTO_SLOW"
            fwd_limit = SWC_SPEED_MID
        else:
            mode = "AUTO_FAST"
            fwd_limit = SWC_SPEED_HIGH

        # CH7 (SWB) control: Reverse speed limit
        raw_swb = self.channels[IBUS_CH_SWB]
        rev_limit = SWB_SPEED_LOW if raw_swb < SWB_THRESH else SWB_SPEED_HIGH

        return Xn, Yn, swd_on, mode, fwd_limit, rev_limit, self._signal_ok

    def close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
