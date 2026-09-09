#!/usr/bin/env python3
# check_stm32_link.py — verifies the RPi <-> STM32 UART link is alive and
# framing correctly, without commanding any motion.
#
# What this DOES prove: the STM32 is powered, flashed with the Phase 5
# firmware, and its feedback frames are reaching the RPi intact (correct
# wiring, correct baud, GPIO14/15 console properly freed).
#
# What this does NOT fully prove: that the RPi's own transmitted command
# frames are reaching the STM32. TX and RX are separate wires on a UART, so
# a healthy RX (this script working at all) is strong evidence the physical
# link as a whole is sound, but not direct proof of the TX half. Full proof
# needs SWB flipped to AUTO on the transmitter and someone reading the
# STM32's own debug telemetry (PA9 -> ESP32 -> USB) or its behavior —
# outside the scope of this bench check, and NOT recommended casually since
# AUTO mode lets this script's commands reach the actuator/motors for real.
#
# This script only ever sends steer=0.0 speed=0.0 — inert if SWB somehow is
# in AUTO (brakes, centers steering) — but it does not verify the STM32
# actually received it. See scratchpad.md for the known gap (no "uart cmd
# ok" bit in the feedback status byte yet).
#
# Usage:
#   source ~/rover-venv/bin/activate
#   python3 check_stm32_link.py

import sys
import time

from uart_link import STM32Link, LINK_TIMEOUT_S


def main():
    link = STM32Link()
    if not link.is_open():
        print("[Check] Could not open the serial port — see the error above.")
        print("        Common causes: wrong device node (try `ls /dev/serial*`), "
              "permission denied (add your user to the 'dialout' group and "
              "re-login), or the getty wasn't actually freed (see "
              "architecture.md §3.6).")
        sys.exit(1)

    print("\nSending neutral command frames (steer=0.0° speed=0.0 m/s) at 20 Hz")
    print("while listening for STM32 feedback frames. Press Ctrl+C to stop.\n")

    t_start = time.monotonic()
    last_print = 0.0
    was_valid = None

    try:
        while True:
            link.send_command(0.0, 0.0)
            d = link.read()
            ok, bad = link.stats()

            now = time.monotonic()
            if now - last_print >= 0.2:
                last_print = now
                elapsed = now - t_start
                rate = ok / elapsed if elapsed > 0 else 0.0
                status_bits = (
                    f"ARMED={'Y' if d.armed else 'n'} "
                    f"AUTO={'Y' if d.auto_active else 'n'} "
                    f"RC_OK={'Y' if d.rc_ok else 'n'} "
                    f"STEER_FAULT={'Y' if d.steer_fault else 'n'}"
                )
                link_str = "CONNECTED" if d.valid else "NO LINK"
                print(
                    f"[{elapsed:6.1f}s] {link_str:10} | frames ok={ok:5d} bad={bad:3d} "
                    f"rate={rate:5.1f}/s | age={d.age_s*1000:6.0f}ms | {status_bits}",
                    end="\r", flush=True,
                )

            if was_valid is True and not d.valid:
                print(f"\n[Check] LINK LOST at t={now - t_start:.1f}s "
                      f"(no valid frame for >{LINK_TIMEOUT_S*1000:.0f}ms)")
            elif was_valid is False and d.valid:
                print(f"\n[Check] Link (re)established at t={now - t_start:.1f}s")
            was_valid = d.valid

            time.sleep(0.05)  # ~20 Hz, matching the STM32 loop rate

    except KeyboardInterrupt:
        pass
    finally:
        ok, bad = link.stats()
        elapsed = time.monotonic() - t_start
        print(f"\n\n[Check] Summary: {ok} good frames, {bad} checksum failures "
              f"over {elapsed:.1f}s ({ok/elapsed if elapsed>0 else 0:.1f} frames/s).")
        if ok == 0:
            print("[Check] FAIL — zero valid frames received. Check: STM32 powered "
                  "and flashed with the Phase 5 firmware, TX/RX not swapped "
                  "(STM32 PA2->RPi Pin8/GPIO14, STM32 PA3<-RPi Pin10/GPIO15), "
                  "common GND connected, baud both sides at 115200.")
        elif bad > ok * 0.05:
            print("[Check] WARNING — significant checksum failure rate; check "
                  "for a marginal/noisy connection or a baud mismatch.")
        else:
            print("[Check] PASS — feedback link (STM32 -> RPi) looks healthy.")
        link.close()


if __name__ == "__main__":
    main()
