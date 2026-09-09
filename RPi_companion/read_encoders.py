#!/usr/bin/env python3
# read_encoders.py — live readout of wheel RPM (and steering angle) as
# reported by the STM32 over the UART link.
#
# The STM32 measures these itself (encoder.c: TIM2/TIM3 quadrature decode,
# ads1115.c: potentiometer feedback) and reports them in every feedback
# frame regardless of MANUAL/AUTO mode — see rpi_link.c. This script just
# listens; it sends nothing, so it's a purely passive readout, safe to run
# at any time including while the rover is being driven manually.
#
# Usage:
#   source ~/rover-venv/bin/activate
#   python3 read_encoders.py

import sys
import time

from uart_link import STM32Link


def main():
    link = STM32Link()
    if not link.is_open():
        print("[Encoders] Could not open the serial port — see the error above.")
        sys.exit(1)

    print("\nListening for encoder + steering feedback from the STM32. "
          "Press Ctrl+C to stop.\n")
    print(f"{'RPM L':>8} {'RPM R':>8} | {'Angle L':>8} {'Angle R':>8} | {'Status':>28} | Link")
    print("-" * 80)

    try:
        while True:
            d = link.read()
            status = (
                f"ARM={'Y' if d.armed else 'n'} "
                f"AUTO={'Y' if d.auto_active else 'n'} "
                f"RC={'Y' if d.rc_ok else 'n'} "
                f"FLT={'Y' if d.steer_fault else 'n'}"
            )
            print(
                f"{d.rpm_L:>+8.2f} {d.rpm_R:>+8.2f} | "
                f"{d.angle_L_deg:>+8.2f} {d.angle_R_deg:>+8.2f} | "
                f"{status:>28} | {'OK' if d.valid else 'LOST'}",
                end="\r", flush=True,
            )
            time.sleep(0.05)  # ~20 Hz, matching the STM32 loop rate
    except KeyboardInterrupt:
        print("\n\n[Encoders] Stopped by user.")
    finally:
        link.close()


if __name__ == "__main__":
    main()
