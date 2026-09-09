"""
interactive_sim.py — Hardware-free simulator.
Imports run_pipeline() directly from pipeline.py.

Usage:
    python interactive_sim.py
"""

import sys, os, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pipeline import run_pipeline
from config import Vmax, Wmax, RPMmax, R, L, DAC_MAX_VALUE, SPOT_RPM, IBUS_CH_SWB, SWB_DEFAULT_SPEED

MAX_DUTY = DAC_MAX_VALUE

# Default speed limits used for 'Run ALL' tests
DEFAULT_FWD_PCT = 1.00   # SWC high  (100 %)
DEFAULT_REV_PCT = 1.00   # SWB high  (80 %) or fallback 100 %

def motor_dir(rpm):
    if abs(rpm) < 0.5: return "STOP"
    return "FWD " if rpm > 0 else "REV "

def draw_bar(value, width=20):
    filled = min(int(abs(value) / RPMmax * width), width)
    bar = "#" * filled + "." * (width - filled)
    return "[" + bar + "]" if value >= 0 else "[" + ("." * (width - filled)) + ("#" * filled) + "]"

def draw_joystick(Xn, Yn, width=21):
    grid = [["·"] * width for _ in range(width)]
    cx = cy = width // 2
    for i in range(width):
        grid[cy][i] = "-"
        grid[i][cx] = "|"
    grid[cy][cx] = "+"
    sx = max(0, min(width - 1, int(cx + Xn * (width // 2))))
    sy = max(0, min(width - 1, int(cy - Yn * (width // 2))))
    grid[sy][sx] = "O"
    lines = ["  +" + "-" * width + "+"]
    for row in grid:
        lines.append("  |" + "".join(row) + "|")
    lines.append("  +" + "-" * width + "+")
    return "\n".join(lines)

def print_result(label, Xn, Yn, fwd_pct=DEFAULT_FWD_PCT, rev_pct=DEFAULT_REV_PCT):
    state, V, W, rL, rR, tL, tR = run_pipeline(Xn, Yn, fwd_pct, rev_pct)
    print(f"\n  {'-'*65}")
    print(f"  {label}  ->  [{state}]")
    print(f"  Input : Xn={Xn:+.3f}  Yn={Yn:+.3f}  "
          f"FWD:{fwd_pct*100:.0f}%  REV:{rev_pct*100:.0f}%")
    print(f"{draw_joystick(Xn, Yn)}")
    if state in ("FWD", "FWD LEFT", "FWD RIGHT", "REV", "REV LEFT", "REV RIGHT"):
        print(f"  V={V:+.4f} m/s   W={W:+.4f} rad/s")
    print(f"  Left  : {draw_bar(rL)}  {rL:>+7.1f} RPM  {motor_dir(rL)}  DAC={tL}")
    print(f"  Right : {draw_bar(rR)}  {rR:>+7.1f} RPM  {motor_dir(rR)}  DAC={tR}")
    print(f"  GPIO  : REV_L={'ON' if rL<0 else 'OFF'}  REV_R={'ON' if rR<0 else 'OFF'}  "
          f"BRAKE={'ON' if state=='BRAKE' else 'OFF'}")
    
    # Sanity checks updated for Pivot Turning
    ok = True
    if state in ("SPOT RIGHT",):
        if not (rL > 0 and rR == 0):
            print("  !! SPOT RIGHT should have rpm_L>0, rpm_R=0"); ok = False
    if state in ("SPOT LEFT",):
        if not (rL == 0 and rR > 0):
            print("  !! SPOT LEFT should have rpm_L=0, rpm_R>0"); ok = False
    if abs(Xn) > 0.1 and state in ("FWD LEFT", "FWD RIGHT", "REV LEFT", "REV RIGHT"):
        if abs(rL - rR) < 0.5:
            print("  !! X axis not affecting motors!"); ok = False
    if ok:
        print("  OK")


# ── Presets ───────────────────────────────────────────────────────────────────
PRESETS = {
    "1":  ("FWD  Full forward",            0.0,    1.0  ),
    "2":  ("FWD  Half forward",            0.0,    0.5  ),
    "3":  ("FWD  Gentle right turn",       0.3,    0.5  ),
    "4":  ("FWD  Sharp  right turn",       1.0,    0.3  ),
    "5":  ("FWD  Gentle left  turn",      -0.3,    0.5  ),
    "6":  ("FWD  Sharp  left  turn",      -1.0,    0.3  ),
    "9":  ("REV  Full reverse",            0.0,   -1.0  ),
    "10": ("REV  Half reverse",            0.0,   -0.5  ),
    "11": ("REV  Gentle right turn",       0.3,   -0.5  ),
    "12": ("REV  Sharp  right turn",       1.0,   -0.3  ),
    "13": ("REV  Gentle left  turn",      -0.3,   -0.5  ),
    "14": ("REV  Sharp  left  turn",      -1.0,   -0.3  ),
    "17": ("SPOT Full  right",             1.0,    0.0  ),
    "18": ("SPOT Half  right",             0.5,    0.0  ),
    "19": ("SPOT Full  left",             -1.0,    0.0  ),
    "20": ("SPOT Half  left",             -0.5,    0.0  ),
    "0":  ("STOP / BRAKE",                 0.0,    0.0  ),
}

def print_menu():
    swb_note = (f"CH{IBUS_CH_SWB+1} (index {IBUS_CH_SWB})"
                if IBUS_CH_SWB >= 0
                else f"NOT ASSIGNED (default {SWB_DEFAULT_SPEED*100:.0f}%)")
    print("\n" + "=" * 67)
    print("  ROBOT SIMULATOR — pipeline.py")
    print(f"  R={R}m  L={L}m  Vmax={Vmax}  Wmax={Wmax}  RPMmax={RPMmax}")
    print(f"  SPOT_RPM={SPOT_RPM} (fixed, no speed-limit scaling)")
    print(f"  SWC → forward speed  (CH5, 3-pos: 30/60/100%)")
    print(f"  SWB → reverse speed  ({swb_note}, 2-pos: 60/100%)")
    print("=" * 67)
    for title, keys in [
        ("FORWARD",   ["1", "2", "3", "4", "5", "6"]),
        ("REVERSE",   ["9", "10", "11", "12", "13", "14"]),
        ("SPOT/STOP", ["17", "18", "19", "20", "0"]),
    ]:
        print(f"\n  -- {title} {'-'*(48-len(title))}")
        for k in keys:
            name, x, y = PRESETS[k]
            print(f"    [{k:>2}]  {name:<38} Xn={x:+.3f}  Yn={y:+.3f}")
    print("\n    [ a]  Run ALL tests at default speed limits")
    print("    [ s]  Run ALL tests across all SWC x SWB combinations")
    print("    [ c]  Custom input with manual speed limits")
    print("    [ m]  Show this menu")
    print("    [ q]  Quit")
    print("-" * 67)

print_menu()
while True:
    try:
        ch = input("\n  Enter choice: ").strip().lower()

        if ch == "q":
            print("  Bye!")
            break
        elif ch == "m":
            print_menu()
        elif ch == "a":
            for k, (name, Xn, Yn) in PRESETS.items():
                print_result(f"[{k:>2}] {name}", Xn, Yn)
        elif ch == "s":
            swc_levels = [("SWC 30%",  0.30), ("SWC 60%", 0.60), ("SWC 100%", 1.00)]
            swb_levels = [("SWB 60%",  0.60), ("SWB 100%", 1.00)]
            for swc_label, fwd in swc_levels:
                for swb_label, rev in swb_levels:
                    print(f"\n  *** {swc_label}  |  {swb_label} ***")
                    for k, (name, Xn, Yn) in PRESETS.items():
                        print_result(f"[{k:>2}] {name}", Xn, Yn, fwd, rev)
        elif ch == "c":
            try:
                Xn  = float(input("  Xn  (-1 to 1): "))
                Yn  = float(input("  Yn  (-1 to 1): "))
                fwd = float(input("  FWD speed limit  0.30/0.60/1.00  [Enter=1.0]: ") or "1.0")
                rev = float(input("  REV speed limit  0.60/1.00  [Enter=1.0]: ") or "1.0")
                print_result("Custom",
                             max(-1.0, min(1.0, Xn)),
                             max(-1.0, min(1.0, Yn)),
                             fwd, rev)
            except ValueError:
                print("  Invalid input — enter numbers only.")
        elif ch in PRESETS:
            name, Xn, Yn = PRESETS[ch]
            print_result(f"[{ch}] {name}", Xn, Yn)
        else:
            print("  Unknown choice. Type 'm' for menu.")

    except KeyboardInterrupt:
        print("\n  Bye!")
        break