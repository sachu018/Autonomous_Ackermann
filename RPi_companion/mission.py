#!/usr/bin/env python3
# mission.py — top-level autonomous mission runner (Phase 5).
#
# Ties together imu.py + uart_link.py + odometry.py + waypoints.py +
# guidance.py into an actual runnable autonomous test. Nothing else in
# RPi_companion/ is a program on its own — this is the one you run.
#
# SAFETY MODEL (already built into the STM32 firmware — this script
# relies on it rather than re-implementing anything): this script ALWAYS
# computes and sends steer/speed commands over the UART link, at 20 Hz,
# regardless of what SWB is set to. The STM32 only ACTS on them while
# SWB=AUTO (rpi_link.c / main.c); while SWB=MANUAL the RC sticks drive the
# rover and these commands are silently ignored. Consequences:
#   - Running this script with SWB=MANUAL is a safe, non-moving dry run —
#     watch the printed pose/command/error output to sanity check
#     tracking before ever letting it actually drive.
#   - Flipping SWB to AUTO engages it; flipping back to MANUAL instantly
#     hands control back to the RC sticks, from anywhere in the mission.
#   - A mission "starts" (odometry POSITION reset to (0,0), current
#     location becomes the path's origin) on the RISING EDGE of SWB going
#     to AUTO — so backing out and re-arming AUTO restarts the mission
#     fresh without restarting this script. The HEADING reference does
#     NOT re-lock on this edge — it's captured exactly once, the first
#     time this script gets a real IMU reading (effectively: whichever way
#     the rover is pointed when you START this script). This is
#     deliberate: it's what makes "compass-referenced" actually mean
#     something across repeated test runs in one session, rather than
#     each restart getting its own arbitrary +X direction — point the
#     rover the way you want the pattern to face BEFORE launching this
#     script, not before each individual AUTO engagement.
#
# Usage:
#   python3 mission.py --pattern straight --length 10
#   python3 mission.py --pattern rectangle --width 8 --height 6
#   python3 mission.py --pattern circle --diameter 6
#   python3 mission.py --pattern lawnmower --row-length 6 --num-rows 4
#   python3 mission.py --csv my_path.csv
#
# Every run automatically archives the path actually used AND a detailed
# telemetry log (pose, commands, Pure Pursuit internals incl. cross-track
# error, full STM32 feedback, raw IMU) to ~/RPi_companion/logs/, both
# sharing one timestamp so they're easy to pair up for plotting/tuning.
# Add --save-csv out.csv to ALSO save a copy of a generated pattern
# somewhere reusable (e.g. to load back in later with --csv) — this is
# optional, separate from the automatic per-run archive.

import argparse
import csv as csv_module
import math
import os
import sys
import time
from datetime import datetime

import rover_config as cfg
import waypoints as wp
from imu import IMU
from uart_link import STM32Link
from odometry import Odometry
from guidance import PurePursuit

LOOP_HZ = 20
LOOP_PERIOD_S = 1.0 / LOOP_HZ


def build_path(args, timestamp):
    """Builds the path AND always archives whatever was actually used
    (generated or loaded) into logs/mission_<timestamp>_path.csv — same
    timestamp as the telemetry log from MissionLogger, so the two files
    for one run are trivially paired up for plotting later. --save-csv is
    for ALSO saving a copy somewhere reusable (e.g. to --csv it back in
    on a later run); it's not what makes the run's own path recoverable."""
    if args.csv:
        path = wp.load_csv(args.csv)
        print(f"[Mission] Loaded {len(path)} waypoints from {args.csv}")
    else:
        generators = {
            "straight": lambda: wp.generate_straight_line(args.length),
            "rectangle": lambda: wp.generate_rectangle(args.width, args.height),
            "circle": lambda: wp.generate_circle(args.diameter),
            "lawnmower": lambda: wp.generate_lawnmower(
                args.row_length, args.num_rows, row_spacing=args.row_spacing),
        }
        path = generators[args.pattern]()
        print(f"[Mission] Generated {len(path)} waypoints for pattern "
              f"'{args.pattern}'")

    log_dir = os.path.expanduser("~/RPi_companion/logs")
    os.makedirs(log_dir, exist_ok=True)
    archive_path = os.path.join(log_dir, f"mission_{timestamp}_path.csv")
    wp.save_csv(path, archive_path)
    print(f"[Mission] Path archived to {archive_path}")

    if args.save_csv:
        wp.save_csv(path, args.save_csv)
        print(f"[Mission] Also saved to {args.save_csv}")

    return path


# Telemetry log columns. Grouped by source so it's easy to find things when
# plotting: pose/commands, Pure Pursuit internals (for tuning Ld/spacing/
# thresholds against real cross-track error), STM32 feedback, raw IMU.
_LOG_COLUMNS = [
    "t_s", "x_m", "y_m", "yaw_deg",
    "steer_cmd_deg", "speed_cmd_ms",
    "cte_m", "alpha_deg", "lookahead_x_m", "lookahead_y_m",
    "target_idx", "mission_finished",
    "angle_L_deg", "angle_R_deg", "rpm_L", "rpm_R",
    "armed", "auto_active", "rc_ok", "steer_fault", "link_ok",
    "imu_heading_deg", "imu_roll_deg", "imu_pitch_deg", "imu_gyro_z_rads",
    "imu_accel_x_ms2", "imu_accel_y_ms2",
    "imu_calib_sys", "imu_calib_gyro", "imu_calib_accel", "imu_calib_mag",
    "imu_valid",
]


class MissionLogger:
    def __init__(self, timestamp, log_dir="~/RPi_companion/logs"):
        log_dir = os.path.expanduser(log_dir)
        os.makedirs(log_dir, exist_ok=True)
        self.path = os.path.join(log_dir, f"mission_{timestamp}.csv")
        self.file = open(self.path, "w", newline="")
        self.writer = csv_module.writer(self.file)
        self.writer.writerow(_LOG_COLUMNS)
        print(f"[Mission] Logging to {self.path}")

    def log(self, t, pose, steer_cmd, speed_cmd, guidance, fb, imu_data):
        la = guidance.last_lookahead or (float("nan"), float("nan"))
        self.writer.writerow([
            f"{t:.3f}", f"{pose.x:.3f}", f"{pose.y:.3f}",
            f"{math.degrees(pose.yaw):.1f}",
            f"{steer_cmd:.2f}", f"{speed_cmd:.3f}",
            f"{guidance.last_cte_m:.3f}" if guidance.last_cte_m is not None else "",
            f"{guidance.last_alpha_deg:.2f}" if guidance.last_alpha_deg is not None else "",
            f"{la[0]:.3f}", f"{la[1]:.3f}",
            guidance.last_target_idx, int(guidance.is_finished()),
            f"{fb.angle_L_deg:.2f}", f"{fb.angle_R_deg:.2f}",
            f"{fb.rpm_L:.2f}", f"{fb.rpm_R:.2f}",
            int(fb.armed), int(fb.auto_active), int(fb.rc_ok),
            int(fb.steer_fault), int(fb.valid),
            f"{imu_data.heading_deg:.2f}" if imu_data else "",
            f"{imu_data.roll_deg:.2f}" if imu_data else "",
            f"{imu_data.pitch_deg:.2f}" if imu_data else "",
            f"{imu_data.gyro_z_rads:.4f}" if imu_data else "",
            f"{imu_data.accel_x_ms2:.3f}" if imu_data else "",
            f"{imu_data.accel_y_ms2:.3f}" if imu_data else "",
            imu_data.calib_sys if imu_data else "",
            imu_data.calib_gyro if imu_data else "",
            imu_data.calib_accel if imu_data else "",
            imu_data.calib_mag if imu_data else "",
            int(imu_data.valid) if imu_data else 0,
        ])

    def close(self):
        self.file.close()


def main():
    parser = argparse.ArgumentParser(description="Autonomous mission runner")
    parser.add_argument("--pattern",
                         choices=["straight", "rectangle", "circle", "lawnmower"])
    parser.add_argument("--csv", help="Load a waypoint CSV instead of generating one")
    parser.add_argument("--save-csv", help="Save the generated pattern to this CSV path")
    parser.add_argument("--length", type=float, default=10.0, help="straight: meters")
    parser.add_argument("--width", type=float, default=8.0, help="rectangle: meters")
    parser.add_argument("--height", type=float, default=6.0, help="rectangle: meters")
    parser.add_argument("--diameter", type=float, default=6.0, help="circle: meters")
    parser.add_argument("--row-length", type=float, default=6.0, help="lawnmower: meters")
    parser.add_argument("--num-rows", type=int, default=4, help="lawnmower: row count")
    parser.add_argument("--row-spacing", type=float, default=cfg.LAWNMOWER_ROW_SPACING_M)
    args = parser.parse_args()

    if not args.csv and not args.pattern:
        parser.error("Specify either --csv <file> or --pattern <name>")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = build_path(args, timestamp)
    if len(path) < 2:
        print("[Mission] Path needs at least 2 waypoints. Aborting.")
        sys.exit(1)

    print("[Mission] Initializing hardware...")
    imu = IMU()
    if not imu.is_healthy():
        print("[Mission] WARNING: IMU not detected/healthy. Dead-reckoning "
              "heading (and therefore x,y) will be meaningless — this run "
              "is only useful for checking the UART link itself, not real "
              "tracking. Fix the IMU before trusting any pose output below.")

    link = STM32Link()
    if not link.is_open():
        print("[Mission] Could not open the STM32 UART link. Aborting.")
        sys.exit(1)

    odo = Odometry(imu, cfg.WHEEL_RADIUS_M)
    guidance = PurePursuit(path)
    logger = MissionLogger(timestamp)

    print("\n[Mission] Ready. SWB=MANUAL is safe — commands are computed "
          "and sent but the STM32 ignores them. Flip SWB to AUTO on the "
          "transmitter to engage. Ctrl+C here to stop.\n")

    t_start = time.monotonic()
    last_tick = t_start
    prev_auto_active = False
    mission_announced_done = False

    try:
        while True:
            now = time.monotonic()
            dt = now - last_tick
            last_tick = now
            if dt <= 0 or dt > 0.5:
                dt = LOOP_PERIOD_S

            fb = link.read()

            if fb.auto_active and not prev_auto_active:
                print("\n[Mission] SWB -> AUTO. Starting/restarting mission "
                      "(position reset to (0,0) here; heading reference "
                      "unchanged from script start).")
                odo.reset()
                guidance = PurePursuit(path)
                mission_announced_done = False
            prev_auto_active = fb.auto_active

            pose = odo.update(fb.rpm_L, fb.rpm_R, dt)

            if guidance.is_finished():
                steer_cmd, speed_cmd = 0.0, 0.0
                if not mission_announced_done:
                    print("\n[Mission] Path complete. Holding (commands "
                          "zeroed). Flip SWB to MANUAL or Ctrl+C to stop.")
                    mission_announced_done = True
            else:
                steer_cmd, speed_cmd = guidance.update(pose)

            link.send_command(steer_cmd, speed_cmd)
            logger.log(now - t_start, pose, steer_cmd, speed_cmd,
                       guidance, fb, odo.last_imu_data)

            cte_str = f"{guidance.last_cte_m:+5.2f}" if guidance.last_cte_m is not None else " ---"
            print(f"\r[{now - t_start:7.1f}s] "
                  f"{'AUTO' if fb.auto_active else 'MANUAL':6} "
                  f"link:{'OK' if fb.valid else 'LOST':4} | "
                  f"pose x={pose.x:+6.2f} y={pose.y:+6.2f} "
                  f"yaw={math.degrees(pose.yaw):+6.1f} | "
                  f"cte={cte_str}m | "
                  f"cmd steer={steer_cmd:+5.1f} spd={speed_cmd:+4.2f} | "
                  f"enc L={fb.rpm_L:+5.1f} R={fb.rpm_R:+5.1f}",
                  end="", flush=True)

            elapsed = time.monotonic() - now
            sleep_t = LOOP_PERIOD_S - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n\n[Mission] Stopped by user.")
    finally:
        link.send_command(0.0, 0.0)  # final safety: neutral command
        time.sleep(0.1)
        link.close()
        logger.close()
        print(f"[Mission] Log saved: {logger.path}")


if __name__ == "__main__":
    main()
