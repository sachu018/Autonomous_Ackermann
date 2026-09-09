#!/usr/bin/env python3
"""
encoder_monitor.py
------------------
Standalone RPM monitor — reads eQEP counters via /sys/bus/counter sysfs.
Kernel: 5.10.168-ti-r83 on BeagleBone Black

Encoders: YT06-OP-600B (600 PPR)
  eQEP1 → counter1 → P8_35 (Phase A) / P8_33 (Phase B) → Motor 1 (Left)
  eQEP2 → counter2 → P8_12 (Phase A) / P8_11 (Phase B) → Motor 2 (Right)

Quadrature: 4 edges per pulse → 600 × 4 = 2400 counts/rev (motor shaft)
With 1:20 gearbox: wheel RPM = motor RPM / 20

Usage:
  python3 encoder_monitor.py             # normal mode
  python3 encoder_monitor.py --raw       # show raw counts too
  python3 encoder_monitor.py --interval 0.5   # set sample interval (seconds)
"""

import time
import os
import sys
import argparse

# ── Constants ────────────────────────────────────────────────────────────────────
PPR                  = 600
QUADRATURE_MULT      = 4
COUNTS_PER_REV       = PPR * QUADRATURE_MULT   # 2400  (motor shaft)
GEAR_RATIO           = 20.0                     # from config.py

ENCODER_COUNTER_MAP = {
    "Motor1-L (eQEP1)": {
        "path":  "/sys/bus/counter/devices/counter1/count0/count",
        "pins":  "P8_35(A) / P8_33(B)",
    },
    "Motor2-R (eQEP2)": {
        "path":  "/sys/bus/counter/devices/counter2/count0/count",
        "pins":  "P8_12(A) / P8_11(B)",
    },
}

# ── Helpers ──────────────────────────────────────────────────────────────────────
def read_count(path: str):
    try:
        with open(path, 'r') as f:
            return int(f.read().strip())
    except Exception:
        return None

def check_paths():
    all_ok = True
    for name, info in ENCODER_COUNTER_MAP.items():
        exists = os.path.exists(info["path"])
        status = "OK" if exists else "MISSING"
        print(f"  [{status}] {name} → {info['path']}")
        if not exists:
            all_ok = False
    return all_ok

def list_all_counters():
    base = "/sys/bus/counter/devices"
    if not os.path.isdir(base):
        print(f"  {base} does not exist — eQEP driver not loaded?")
        return
    for dev in sorted(os.listdir(base)):
        count_path = os.path.join(base, dev, "count0", "count")
        name_path  = os.path.join(base, dev, "name")
        name = open(name_path).read().strip() if os.path.exists(name_path) else "?"
        val  = read_count(count_path) if os.path.exists(count_path) else "N/A"
        print(f"  {dev}  name={name}  count={val}  path={count_path}")

# ── Main ─────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Encoder RPM monitor for BeagleBone eQEP")
    parser.add_argument("--raw",      action="store_true", help="Show raw counts alongside RPM")
    parser.add_argument("--interval", type=float, default=0.1, help="Sample interval in seconds (default 0.1)")
    parser.add_argument("--list",     action="store_true", help="List all counter devices and exit")
    args = parser.parse_args()

    if args.list:
        print("\nAvailable counter devices:")
        list_all_counters()
        return

    print("\n── Encoder Monitor ──────────────────────────────────────────────────")
    print(f"  PPR={PPR}  Quadrature×{QUADRATURE_MULT}  Counts/rev={COUNTS_PER_REV}  Gear={GEAR_RATIO}:1")
    print(f"  Sample interval: {args.interval*1000:.0f} ms\n")
    print("  Counter path check:")
    if not check_paths():
        print("\n  WARNING: Some counter paths missing. Run with --list to see available counters.")
        print("  The encoder overlay may not have loaded. Check /boot/uEnv.txt for:")
        print("    uboot_overlay_addr5=/lib/firmware/bone_eqep1-00A0.dtbo")
        print("    uboot_overlay_addr6=/lib/firmware/bone_eqep2-00A0.dtbo\n")
    print("─" * 72)
    print(f"  {'Name':<22} {'Motor RPM':>12} {'Wheel RPM':>12} {'Dir':>5}", end="")
    if args.raw:
        print(f"  {'Δcounts':>9}", end="")
    print()
    print("─" * 72)

    # Initialise
    prev   = {name: read_count(info["path"]) for name, info in ENCODER_COUNTER_MAP.items()}
    t_prev = time.monotonic()

    try:
        while True:
            time.sleep(args.interval)
            now = time.monotonic()
            dt  = now - t_prev
            t_prev = now

            rows = []
            for name, info in ENCODER_COUNTER_MAP.items():
                curr = read_count(info["path"])
                if curr is None or prev[name] is None:
                    rows.append((name, None, None, None, None))
                    prev[name] = curr
                    continue

                delta = curr - prev[name]
                prev[name] = curr

                # 32-bit rollover correction
                if delta >  2**31: delta -= 2**32
                if delta < -2**31: delta += 2**32

                wheel_rpm = (delta / (COUNTS_PER_REV * dt)) * 60.0
                motor_rpm = wheel_rpm * GEAR_RATIO
                direction = "FWD" if motor_rpm > 0.5 else ("REV" if motor_rpm < -0.5 else "STOP")
                rows.append((name, motor_rpm, wheel_rpm, direction, delta))

            # Print on one refreshing block
            lines = []
            for name, motor_rpm, wheel_rpm, direction, delta in rows:
                if motor_rpm is None:
                    lines.append(f"  {name:<22} {'ERROR':>12} {'ERROR':>12} {'?':>5}")
                else:
                    line = (f"  {name:<22} {motor_rpm:>+11.1f} {wheel_rpm:>+11.2f} {direction:>5}")
                    if args.raw:
                        line += f"  {delta:>+9d}"
                    lines.append(line)

            # Move cursor up to overwrite previous output
            print(f"\033[{len(rows)}A", end="")
            for line in lines:
                print(f"\033[2K{line}")

    except KeyboardInterrupt:
        print("\n── Stopped ──")

if __name__ == "__main__":
    main()