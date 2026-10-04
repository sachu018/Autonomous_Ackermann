# Rover System — Complete Technical Reference

> **Author:** Auto-generated from BBB-backup-2026-07-01  
> **Purpose:** Give a new project engineer complete understanding of the rover system — hardware, software, Linux integration, and safety — so they can modify, extend, or debug it independently.

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Hardware Architecture](#2-hardware-architecture)
3. [BeagleBone Black Pin Mapping](#3-beaglebone-black-pin-mapping)
4. [Linux Configuration](#4-linux-configuration)
5. [Software Architecture](#5-software-architecture)
6. [Control Pipeline — End to End](#6-control-pipeline--end-to-end)
7. [Startup Sequence](#7-startup-sequence)
8. [Safety Systems](#8-safety-systems)
9. [Known Hardware Details & Unknowns](#9-known-hardware-details--unknowns)
10. [How to Add a New Feature](#10-how-to-add-a-new-feature)
11. [Troubleshooting Guide](#11-troubleshooting-guide)
12. [File Reference](#12-file-reference)

---

## 1. System Overview

### What is this?

An **agricultural autonomous rover** with differential drive (two rear BLDC motors, two front castor wheels). Currently controlled manually via a **FlySky FS-i6X RC transmitter**. The goal is to eventually extend to full autonomous waypoint following.

### Controller

- **Board:** BeagleBone Black (BBB) — AM335x 1GHz ARM Cortex-A8
- **OS:** Debian Bookworm, kernel `5.10.168-ti-r83`
- **Python:** 3.11.2 (virtual environment at `/home/debian/Robot/env/`)

### Block Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                         FlySky FS-i6X Transmitter                    │
│                   (CH1=Steering, CH2=Throttle,                      │
│                    CH5=SWC Speed, CH6=SWD Arm, CH7=SWB Rev Limit)   │
└─────────────────────────┬───────────────────────────────────────────┘
                          │ 2.4GHz RC link
                          ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    FlySky FS-iA6B Receiver (iBUS)                    │
│                         UART output (115200 baud)                    │
└─────────────────────────┬───────────────────────────────────────────┘
                          │ UART5 — P8_37(TX), P8_38(RX) — /dev/ttyS5
                          ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    BeagleBone Black (Linux Debian)                    │
│                                                                      │
│  main.py ─── 20Hz control loop                                       │
│    ├── flysky_receiver.py ── reads iBUS frames from UART5            │
│    ├── pipeline.py ────────── steering + kinematics                  │
│    ├── motor_controller.py ── writes DACs + GPIOs                    │
│    ├── motor_controller_closed_loop.py ── optional PID speed control │
│    └── kinematics.py ─────── diff-drive math (V, W → RPM_L, RPM_R)  │
│                                                                      │
│  GPIO outputs:                                                       │
│    P8_14 ── Contactor relay (main power to motor controllers)        │
│    P8_7  ── REV_LEFT (reverse direction select)                     │
│    P8_9  ── REV_RIGHT                                                │
│    P8_8  ── BRAKE_LEFT (brake input)                                │
│    P8_10 ── BRAKE_RIGHT                                              │
│                                                                      │
│  I2C Bus 2 (P9_19=SCL, P9_20=SDA):                                  │
│    addr 0x60 ── DAC R (right motor throttle voltage 0-5V)           │
│    addr 0x61 ── DAC L (left motor throttle voltage 0-5V)            │
│    addr 0x28 ── BNO055 IMU (9-axis absolute orientation)            │
│                                                                      │
│  eQEP (encoder counters via sysfs):                                  │
│    counter1 ── eQEP1, P8_35(CLK)/P8_33(STR), Left motor             │
│    counter2 ── eQEP2b, P8_12(CLK)/P8_11(STR), Right motor           │
└─────┬─────────────────────┬─────────────────────────────────────────┘
      │ DAC outputs (I2C)   │ GPIO outputs
      ▼                     ▼
┌──────────────────┐  ┌──────────────────────────────────────────────┐
│  MCP4725 DAC L    │  │  Optocoupler + Level Shifter Board           │
│  (addr 0x61)      │  │                                              │
│  0-5V analog out  │  │  GPIO → optocoupler → controller BRAKE input │
└────────┬──────────┘  │  GPIO → optocoupler → controller REV input   │
         │             └──────────────────┬───────────────────────────┘
         ▼                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│                     BLDC Motor Controller (E-Scooter type)           │
│                                                                      │
│  LEFT controller:                                                    │
│    Throttle in: 0-5V (from DAC L via level shifter)                 │
│    Brake in:    GND when active (from optocoupler)                   │
│    Reverse in:  GND when active (from optocoupler)                   │
│    Motor out:   3-phase BLDC to LEFT wheel                           │
│                                                                      │
│  RIGHT controller:                                                   │
│    Throttle in: 0-5V (from DAC R via level shifter)                 │
│    Brake in:    GND when active (from optocoupler)                   │
│    Reverse in:  GND when active (from optocoupler)                   │
│    Motor out:   3-phase BLDC to RIGHT wheel                          │
└──────────────────────┬──────────────────────────────────────────────┘
                       ▼
┌─────────────────────────────────────────────────────────────────────┐
│  Mechanical:                                                        │
│    Rear: 2× BLDC motors (400 RPM max) → 1:20 gearbox → wheels      │
│    Front: 2× castor wheels (free-rotating)                          │
│    Wheel radius: 0.175 m                                            │
│    Track width:  0.6 m                                              │
│    Max wheel RPM: 20 (after gearbox)                                │
│    Max linear velocity: 0.366 m/s                                   │
│    Encoders: YT06-OP-600B (600 PPR, quadrature = 2400 cnt/rev)     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 2. Hardware Architecture

### 2.1 Drive System

| Parameter | Value | Source |
|-----------|-------|--------|
| Wheel radius (`R`) | 0.175 m | `config.py:4` |
| Track width (`L`) | 0.6 m | `config.py:5` |
| Max motor RPM | 400 | `config.py:9` |
| Gearbox ratio | 20:1 | `config.py:10` |
| Max wheel RPM | 20 | `config.py:12` (400/20) |
| Max linear velocity (`Vmax`) | 0.366 m/s | `config.py:6` |
| Max angular velocity (`Wmax`) | 1.22 rad/s | `config.py:7` |
| Spot turn RPM | 8.75 (wheel) | `config.py:28` |

### 2.2 Motor Controller Interface (E-Scooter BLDC Controllers)

Each motor controller has three control inputs:

| Input | Signal Type | Active State | Driven By |
|-------|-------------|--------------|-----------|
| **Throttle** | Analog voltage | 0-5V (0=stop, 5V=max) | DAC (via level shifter) |
| **Brake** | Digital (GND pull) | Connect to GND = brake ON | GPIO → optocoupler (open drain) |
| **Reverse** | Digital (GND pull) | Connect to GND = reverse ON | GPIO → optocoupler (open drain) |

**Critical detail:** Both the brake and reverse inputs on the motor controller work by pulling the input pin to GND. The BeagleBone GPIOs drive optocouplers which sink the controller input to GND when active. This means:
- GPIO HIGH → optocoupler ON → controller input connected to GND → **active**
- GPIO LOW → optocoupler OFF → controller input floating → **inactive**

**Direction logic:** When reverse is OFF (floating), the motor runs forward with throttle. When reverse is ON (GND), the motor runs backward with throttle.

### 2.3 DAC System (Throttle Generation)

The BeagleBone generates the 0-5V analog throttle signals using **I2C DACs** (MCP4725 or compatible):

- **DAC L** (left motor): I2C address `0x61`
- **DAC R** (right motor): I2C address `0x60`
- Both on **I2C Bus 2** (P9_19 SCL, P9_20 SDA)
- 12-bit resolution: 0-4095 → 0-5V analog output
- Level shifter board between DAC output and motor controller input

**DAC write protocol** (from `motor_controller.py:57-64`):
```python
# 12-bit value split into two bytes
upper = (value >> 4) & 0xFF    # top 8 bits
lower = (value << 4) & 0xFF    # bottom 4 bits (shifted to nibble)
bus.write_i2c_block_data(addr, 0x40, [upper, lower])
```

The command byte `0x40` means "write DAC register, no EEPROM write" (standard MCP4725 fast write).

### 2.4 FlySky RC System

- **Transmitter:** FlySky FS-i6X (6+ channel)
- **Receiver:** FS-iA6B (iBUS protocol)
- **Connection:** UART5 at 115200 baud, `/dev/ttyS5`
- **Pins:** P8_37 (TX), P8_38 (RX)
- **Protocol:** iBUS — 32-byte frames at ~7ms intervals

**Channel mapping** (from `config.py:42-48`):

| Channel | Index | Control | Function |
|---------|-------|---------|----------|
| CH1 | 0 | `Xn` | Steering (Left/Right) |
| CH2 | 1 | `Yn` | Throttle (Forward/Reverse) |
| CH5 | 4 | `SWC` | 3-position switch → forward speed limit |
| CH6 | 5 | `SWD` | 2-position switch → arm/disarm contactor |
| CH7 | 6 | `SWB` | 2-position switch → reverse speed limit |

**Stick range:** 1000-1500-2000 (min-center-max)  
**Deadband:** 0.05 (±5% around center)  
**RC timeout:** 0.4 seconds — if no valid frame for 400ms, signal is considered lost

**Speed limit switch positions:**

| Switch | Position | Raw Value | Speed Limit |
|--------|----------|-----------|-------------|
| SWC (CH5) | Low | <1250 | 30% forward |
| SWC | Mid | 1250-1750 | 60% forward |
| SWC | High | >1750 | 100% forward |
| SWB (CH7) | Low | <1500 | 60% reverse |
| SWB | High | >1500 | 100% reverse |

### 2.5 Encoder System (eQEP)

- **Encoder model:** YT06-OP-600B (600 pulses per revolution)
- **Decoding:** Quadrature (4× edges per pulse) = 2400 counts/rev
- **Location:** Motor shaft (before gearbox)
- **Interface:** TI eQEP peripheral → Linux counter subsystem (sysfs)

| Motor | eQEP | Counter Device | Phase A | Phase B |
|-------|------|---------------|---------|---------|
| Left | eQEP1 | `/sys/bus/counter/devices/counter1/count0/count` | P8_35 | P8_33 |
| Right | eQEP2b | `/sys/bus/counter/devices/counter2/count0/count` | P8_12 | P8_11 |

**RPM calculation** (from `main.py:95-119`):
```python
delta = curr - prev
# Handle 32-bit rollover
if delta > 2**31: delta -= 2**32
if delta < -2**31: delta += 2**32
rpm = (delta / (2400 * dt)) * 60.0    # motor shaft RPM (before gearbox)
```

### 2.6 IMU (BNO055)

- **Sensor:** Bosch BNO055 — 9-axis absolute orientation sensor
- **Interface:** I2C Bus 2 (shared with DACs)
- **Address:** 0x28 (standard)
- **Library:** Adafruit CircuitPython BNO055 driver
- **Features:** On-chip fusion → Euler angles, gravity-compensated accel
- **Calibration:** Saved to `/home/debian/Robot/bno055_calibration.json`

**Optional:** The system runs without IMU if the BNO055 is not connected — it degrades gracefully.

### 2.7 Power and Safety

- **Main contactor:** GPIO-controlled relay (P8_14, GPIO 26) that enables/disables power to the motor controllers
- **Contact state:** HIGH = engaged (motors can run), LOW = disengaged (motors physically disconnected from power)
- **Arming:** Only happens when SWD (CH6) on the transmitter is ON (>1450 raw) and debounced for 500ms
- **Signal loss:** If RC signal is lost for >400ms, motors stop immediately
- **Startup state:** Disarmed — must arm via SWD before any motion

---

## 3. BeagleBone Black Pin Mapping

### 3.1 GPIO Pins

| Function | P9/P8 Pin | GPIO # | Sysfs Export | Direction | Logic |
|----------|-----------|--------|--------------|-----------|-------|
| CONTACTOR | P8_14 | 26 | `/sys/class/gpio/gpio26` | OUT | HIGH=engaged, LOW=disengaged |
| REV_LEFT | P8_7 | 66 | `/sys/class/gpio/gpio66` | OUT | HIGH=reverse ON, LOW=forward |
| BRAKE_LEFT | P8_8 | 67 | `/sys/class/gpio/gpio67` | OUT | HIGH=brake ON, LOW=released |
| REV_RIGHT | P8_9 | 69 | `/sys/class/gpio/gpio69` | OUT | HIGH=reverse ON, LOW=forward |
| BRAKE_RIGHT | P8_10 | 68 | `/sys/class/gpio/gpio68` | OUT | HIGH=brake ON, LOW=released |

**Important:** These GPIOs drive optocouplers. When the GPIO goes HIGH, the optocoupler turns ON, which connects the motor controller's BRAKE or REV input to GND (activating that function).

### 3.2 I2C Pins

| Bus | Pin | Function | Devices |
|-----|-----|----------|---------|
| I2C2 | P9_19 | SCL (clock) | DAC L (0x61), DAC R (0x60), BNO055 (0x28) |
| I2C2 | P9_20 | SDA (data) | Same as above |

Bus number: **2** (opened as `smbus2.SMBus(2)` in `motor_controller.py:32`)

### 3.3 UART Pins

| UART | Pin | Function | Device |
|------|-----|----------|--------|
| UART5 | P8_37 | TX (from BBB) | FlySky FS-iA6B receiver |
| UART5 | P8_38 | RX (to BBB) | FlySky FS-iA6B receiver |

Device node: `/dev/ttyS5`, 115200 baud, 8N1

### 3.4 eQEP (Encoder) Pins

| eQEP | Counter | Pin A (CLK) | Pin B (STR) | Motor |
|------|---------|-------------|-------------|-------|
| eQEP1 | counter1 | P8_35 | P8_33 | Left |
| eQEP2b | counter2 | P8_12 | P8_11 | Right |

### 3.5 Pinmux Configuration (from `startup.sh`)

```bash
# Applied at boot by uEnv.txt overlays (no manual config-pin needed):
#   UART5  (P8_37/P8_38)
#   eQEP1  (P8_35/P8_33)
#   eQEP2b (P8_12/P8_11)

# Applied by startup.sh via config-pin:
config-pin P8_7  gpio    # REV_LEFT
config-pin P8_8  gpio    # BRAKE_LEFT
config-pin P8_9  gpio    # REV_RIGHT
config-pin P8_10 gpio    # BRAKE_RIGHT
config-pin P8_14 gpio    # CONTACTOR
config-pin P9_19 i2c     # I2C2_SCL
config-pin P9_20 i2c     # I2C2_SDA
```

---

## 4. Linux Configuration

### 4.1 Boot Configuration (`/boot/uEnv.txt`)

```bash
uname_r=5.10.168-ti-r83          # Kernel version
enable_uboot_overlays=1           # Enable U-Boot overlay loading
uboot_overlay_addr4=/lib/firmware/BB-UART5-00A0.dtbo
uboot_overlay_addr5=/lib/firmware/bone_eqep1-00A0.dtbo
uboot_overlay_addr6=/lib/firmware/bone_eqep2b-00A0.dtbo
disable_uboot_overlay_video=1     # HDMI disabled (saves pins)
disable_uboot_overlay_audio=1     # Audio disabled
uboot_overlay_pru=AM335X-PRU-UIO-00A0.dtbo   # PRU in UIO mode
enable_uboot_cape_universal=1     # Universal cape support
```

### 4.2 File System (`/etc/fstab`)

```
/dev/mmcblk0p3  /          ext4  noatime,errors=remount-ro  0  1
/dev/mmcblk0p1  /boot/firmware  vfat  user,uid=1000,gid=1000  0  2
/dev/mmcblk0p2  none       swap  sw                           0  0
debugfs         /sys/kernel/debug  debugfs  defaults          0  0
```

### 4.3 Kernel Modules (`/etc/modules`)

Empty — no additional modules loaded at boot. eQEP/UART/I2C support is built into the kernel or loaded via DT overlays.

### 4.4 Service: `robot.service`

```ini
[Unit]
Description=Robot Motor Controller
After=network.target

[Service]
Type=simple
User=debian
WorkingDirectory=/home/debian/Robot
ExecStart=/bin/bash /home/debian/Robot/startup.sh
Restart=on-failure
RestartSec=5
StartLimitIntervalSec=60
StartLimitBurst=3

[Install]
WantedBy=multi-user.target
```

- **What it does:** Starts `startup.sh` as user `debian` on boot
- **Restart policy:** Restarts on failure (max 3 times in 60 seconds, 5s between attempts)
- **Symlink:** Also present as `Robot.service` and `rover.service` (older copies)

---

## 5. Software Architecture

### 5.1 Module Dependency Graph

```
main.py (entry point, 20Hz control loop)
├── config.py                   (all constants, pin numbers, addresses)
├── pipeline.py                 (control logic: steering → RPM)
│   └── utils/kinematics.py     (differential drive math)
├── drivers/flysky_receiver.py  (iBUS protocol over UART)
├── drivers/motor_controller.py (DAC + GPIO low-level control)
└── drivers/motor_controller_closed_loop.py  (optional PID wrapper)
```

### 5.2 Supporting Files

| File | Purpose |
|------|---------|
| `interactive_sim.py` | Hardware-free simulation of `pipeline.py` — test steering logic |
| `teleop_key.py` | Keyboard teleoperation over SSH (for testing without RC) |
| `visualize_imu.py` | Web dashboard — 3D chassis view + real-time charts (SSE) |
| `dashboard.py` | Older version of web dashboard |
| `encoder_monitor.py` | Standalone RPM monitor from eQEP counters |
| `monitor_raw_counts.py` | Debug script — raw eQEP count polling |
| `test_pins.py` | Hardware test — sets GPIOs HIGH for multimeter check |
| `diagnostic_wheels.py` | Direct motor test — writes DAC values without main loop |
| `ugv_logger.py` | CSV data logger for post-run analysis |
| `setup_service.py` | Installs `robot.service` into `/etc/systemd/system/` |
| `stop_service.py` | Stops the `robot.service` |

### 5.3 Data Flow (Execution Path)

```
FlySky Transmitter sticks
        │
        ▼ [2.4GHz RF]
FlySky FS-iA6B Receiver
        │
        ▼ [UART5, 115200 baud, iBUS protocol]
flysky_receiver.py:read()
  ├── _read_frame()       ← parse 32-byte iBUS frame, validate checksum
  ├── _to_float(raw)      ← convert 1000-2000 → -1.0 to +1.0 (with deadband)
  ├── _swc_to_speed()     ← 3-position → 0.30 / 0.60 / 1.00
  └── _swb_to_speed()     ← 2-position → 0.60 / 1.00
        │
        ▼ [returns: Xn, Yn, fwd_safe_pct, rev_safe_pct, swd_on, rc_ok]
main.py (20Hz loop)
  │
  ├── 1. Read receiver          → Xn, Yn, swd_on, rc_ok, etc.
  ├── 2. _handle_contactor()    → edge-detect + debounce on swd_on
  │     └── motors.energize_system() / deenergize_system()
  ├── 3. Check motors_permitted = motors.armed AND rc_ok
  │
  ├── 4. IF permitted:
  │     └── run_pipeline(Xn, Yn, fwd_safe_pct, rev_safe_pct)
  │           ├── Check BRAKE condition (neutral sticks or no RC)
  │           ├── Check SPOT TURN (Yn=0, Xn≠0)
  │           │     └── return SPOT_RPM to one wheel, 0 to other
  │           ├── Differential drive:
  │           │     ├── V = Yn * Vmax                     ← linear velocity
  │           │     ├── W_scale = (|Xn|³)/(|Yn|+ε) * ε   ← speed-scaled steering
  │           │     ├── W = copysign(W_scale, Xn) * Wmax  ← angular velocity
  │           │     ├── _apply_safety(V, W, safe_pct)     ← speed limit
  │           │     └── compute_rpm(V, W, WHEEL_RPM_MAX)  ← kinematics
  │           └── _to_throttle(rpm)                       ← RPM → DAC value
  │                 └── int((abs(rpm) / WHEEL_RPM_MAX) * 4095)
  │     ↓ returns: state, V, W, rpm_L, rpm_R, thr_L, thr_R
  │
  ├── 5. Drive motors:
  │     ├── BRAKE        → motors.brake()       (DAC=0, brake GPIO=HIGH)
  │     ├── DISARMED/NO SIGNAL → motors.stop()  (DAC=0, all GPIO=LOW)
  │     ├── REV states   → brake_then_reverse() (brake 100ms, then reverse)
  │     └── FWD states   → set_motors()         (DAC + GPIO)
  │           ├── Direction change? → DAC=0 → toggle REV GPIOs → wait 80ms
  │           └── set REV GPIOs, release BRAKE GPIOs, write DAC values
  │
  ├── 6. _update_encoders()    ← read eQEP counters, compute RPM
  ├── 7. _send_telemetry()     ← UDP packet to localhost:5005 (for web UI)
  └── 8. Rate-limit to 20Hz   ← time.sleep(DT - elapsed)
```

### 5.4 Threading Model

- **Single-threaded** — the main loop in `main.py` runs sequentially
- **Exception:** `visualize_imu.py` runs as a **separate process** (launched from `startup.sh:78`), which starts its own threads:
  - Thread 1: UDP listener for telemetry + BNO055 polling
  - Thread 2: HTTP server (SSE + REST) for the web dashboard
- The main control loop is NOT multithreaded — no thread safety concerns in the critical path

---

## 6. Control Pipeline — End to End

```
FlySky Stick Position
   CH1 = 0.3 (right), CH2 = 0.5 (half forward)
        │
        ▼
[1] flysky_receiver.py:read()
    → Xn = +0.30, Yn = +0.50
    → fwd_safe_pct = 0.60 (SWC in mid position)
    → rev_safe_pct = 1.00 (SWB in high position)
    → swd_on = True, rc_ok = True
        │
        ▼
[2] main.py:_handle_contactor(True)
    → Rising edge detected, debounce OK
    → motors.energize_system()
    → GPIO P8_14 = HIGH (contactor closes, power to motor controllers)
        │
        ▼
[3] run_pipeline(Xn=0.30, Yn=0.50, fwd=0.60, rev=1.00, rc_ok=True)
    │
    ├─ Not BRAKE (sticks not neutral)
    ├─ Not SPOT (Yn ≠ 0)
    ├─ going_fwd = True (Yn > 0)
    ├─ safe_pct = 0.60
    │
    ├─ V = 0.50 × 0.366 m/s = 0.183 m/s
    │
    ├─ W_scale = (0.30³) / (0.50 + 0.30) × 0.30
    │         = 0.027 / 0.80 × 0.30
    │         = 0.0101
    ├─ W = +0.0101 × 1.22 = +0.0124 rad/s
    │
    ├─ _apply_safety(V=0.183, W=0.0124, safe=0.60)
    │   v_limit = 0.366 × 0.60 = 0.2196
    │   V = min(0.183, 0.2196) = 0.183 (unchanged)
    │
    ├─ compute_rpm(V=0.183, W=0.0124, limit=20.0)
    │   ├─ VL = 0.183 - (0.6/2) × 0.0124 = 0.183 - 0.0037 = 0.1793
    │   ├─ VR = 0.183 + 0.0037 = 0.1867
    │   ├─ rpm_L = (0.1793 / (2π × 0.175)) × 60 = 9.79
    │   ├─ rpm_R = (0.1867 / (2π × 0.175)) × 60 = 10.20
    │   ├─ peak = 10.20 < 20.0 (no scaling needed)
    │   ├─ MIN_RATIO check: faster=10.20, slower_min=2.04
    │   │   rpm_L=9.79 > 2.04 ✓, rpm_R=10.20 > 2.04 ✓
    │   └─ return rpm_L=9.79, rpm_R=10.20
    │
    ├─ _to_throttle(9.79)  → int((9.79/20.0) × 4095) = int(2004) = 2004
    ├─ _to_throttle(10.20) → int((10.20/20.0) × 4095) = int(2088) = 2088
    │
    └─ return "FWD RIGHT", V=0.183, W=0.0124, rpm_L=9.79, rpm_R=10.20,
             thr_L=2004, thr_R=2088
        │
        ▼
[4] motors.set_motors(rpm_L=9.79, rpm_R=10.20, dac_L=2004, dac_R=2088)
    │
    ├─ rpm_L_mod = 9.79 × 1 = 9.79 (LEFT_MOTOR_DIR = 1)
    ├─ rpm_R_mod = 10.20 × 1 = 10.20
    ├─ new_dir_L = rpm_L < 0 → False (forward)
    ├─ new_dir_R = rpm_R < 0 → False (forward)
    ├─ dir_changed = (False != last_dir_L) or (False != last_dir_R) = False
    │
    ├─ REV_LEFT GPIO = LOW, REV_RIGHT GPIO = LOW  (forward direction)
    ├─ BRAKE_LEFT GPIO = LOW, BRAKE_RIGHT GPIO = LOW (brakes off)
    ├─ wait 20ms (MOTOR_SEQ_DELAY)
    │
    ├─ I2C write to DAC L (0x61): value 2004
    │   → upper = (2004 >> 4) & 0xFF = 0x7D
    │   → lower = (2004 << 4) & 0xFF = 0x40
    │   → bus.write_i2c_block_data(0x61, 0x40, [0x7D, 0x40])
    │
    ├─ I2C write to DAC R (0x60): value 2088
    │   → upper = (2088 >> 4) & 0xFF = 0x82
    │   → lower = (2088 << 4) & 0xFF = 0x80
    │   → bus.write_i2c_block_data(0x60, 0x40, [0x82, 0x80])
    │
    ▼
[5] DAC outputs: L = ~2.45V, R = ~2.55V
    │
    ▼ [Level shifters]
[6] Motor controller throttle inputs: L = ~2.45V, R = ~2.55V
    │
    ▼ [BLDC controller interprets voltage as speed command]
[7] Left motor spins forward at ~49% speed
    Right motor spins forward at ~51% speed
    → Rover moves forward and slightly right (differential steering)
```

---

## 7. Startup Sequence

### Power-On to main.py

```
Step 1: Power applied
  └─ ROM bootloader loads from eMMC/SD card

Step 2: U-Boot (secondary bootloader)
  └─ Reads /boot/uEnv.txt
     ├─ Loads kernel: vmlinuz-5.10.168-ti-r83
     ├─ Loads Device Tree: am335x-boneblack-uboot-univ.dtb
     ├─ Applies overlays (uEnv.txt lines 19-21):
     │   ├─ BB-UART5-00A0.dtbo    → UART5 on P8_37/P8_38
     │   ├─ bone_eqep1-00A0.dtbo  → eQEP1 on P8_35/P8_33
     │   └─ bone_eqep2b-00A0.dtbo → eQEP2b on P8_12/P8_11
     ├─ Disables HDMI (saves pins)
     ├─ Disables audio
     └─ Boots kernel

Step 3: Kernel boots
  ├─ Mounts rootfs from /dev/mmcblk0p3 (ext4)
  ├─ Mounts /boot/firmware from /dev/mmcblk0p1 (vfat)
  ├─ Initializes hardware drivers:
  │   ├─ I2C2 (P9_19/P9_20) — built-in
  │   ├─ UART5 (P8_37/P8_38) — from overlay
  │   ├─ eQEP1, eQEP2 — from overlays
  │   └─ GPIO modules
  └─ Starts systemd (PID 1)

Step 4: systemd starts services
  ├─ Enables multi-user.target
  │   └─ robot.service is WantedBy=multi-user.target
  │
  └─ Executes robot.service:
      User=debian
      ExecStart=/bin/bash /home/debian/Robot/startup.sh

Step 5: startup.sh runs
  │
  ├─ export BLINKA_FORCEBOARD=BEAGLEBONE_BLACK
  │   (Tells Blinka library: "act like a BBB" — needed since EEPROM
  │    checks may fail in some cases)
  │
  ├─ config-pin P8_7  gpio     (REV_LEFT)
  ├─ config-pin P8_8  gpio     (BRAKE_LEFT)
  ├─ config-pin P8_9  gpio     (REV_RIGHT)
  ├─ config-pin P8_10 gpio     (BRAKE_RIGHT)
  ├─ config-pin P8_14 gpio     (CONTACTOR)
  ├─ config-pin P9_19 i2c      (I2C2_SCL)
  ├─ config-pin P9_20 i2c      (I2C2_SDA)
  │
  ├─ Export GPIOs to sysfs:
  │   echo 26 > /sys/class/gpio/export     (P8_14)
  │   echo 66 > /sys/class/gpio/export     (P8_7)
  │   echo 67 > /sys/class/gpio/export     (P8_8)
  │   echo 68 > /sys/class/gpio/export     (P8_10)
  │   echo 69 > /sys/class/gpio/export     (P8_9)
  │
  ├─ Enable eQEP counters:
  │   echo 4294967295 > counter1/count0/ceiling
  │   echo 4294967295 > counter2/count0/ceiling
  │   echo 1 > counter1/count0/enable
  │   echo 1 > counter2/count0/enable
  │
  ├─ Create log directory, prune old logs (keep 10)
  │
  ├─ Launch visualizer (background):
  │   python3 visualize_imu.py &
  │
  └─ Launch main.py (foreground, replaces bash via exec):
      exec python3 main.py

Step 6: main.py starts
  ├─ Creates FlySkyReceiver()    → opens /dev/ttyS5
  ├─ Creates MotorController()   → opens I2C bus 2, sets up GPIOs
  ├─ Initializes eQEP encoders
  ├─ Attempts BNO055 IMU init
  ├─ Sets up signal handlers (SIGINT, SIGTERM → safe shutdown)
  └─ Enters 20Hz control loop
```

---

## 8. Safety Systems

### 8.1 Physical Safety

| Safety Feature | Mechanism | File |
|---------------|-----------|------|
| Main contactor | Relay on P8_14 disconnects motor controller power | `motor_controller.py:42-55` |
| Contactor arming | Only via SWD switch — edge-detect + 500ms debounce | `main.py:277-296` |
| Disarmed by default | Contactor OFF at startup — must arm manually | `main.py:299-300` |

### 8.2 Software Safety

| Safety Feature | Mechanism | File |
|---------------|-----------|------|
| RC signal loss | 400ms timeout → forces BRAKE state | `flysky_receiver.py:87-89` |
| Both sticks neutral | Yn=0 AND Xn=0 → BRAKE state | `pipeline.py:18-19` |
| No RC → no motion | `motors_permitted = motors.armed AND rc_ok` | `main.py:312` |
| Speed limits (SWC/SWB) | Transmitter switches limit max speed | `flysky_receiver.py:67-74` |
| Shutdown signal handler | SIGINT/SIGTERM → brake + deenergize + close serial | `main.py:263-271` |
| Override via DISARMED | SWD off → contactor opens → motors stop | `main.py:291-294` |
| Reverse brake sequence | 100ms brake before engaging reverse | `motor_controller.py:128-143` |

### 8.3 Startup Safety

- The robot starts **DISARMED** — the contactor is open, no power to motor controllers
- The console prints: *"Arm the robot by flipping SWD (CH6) to ON"*
- If the transmitter is off or out of range, `rc_ok=False` and motors stay stopped
- The contactor debounce prevents accidental arming from switch bounce

---

## 9. Known Hardware Details & Unknowns

### 9.1 Verified from Code

| Item | Detail | Source |
|------|--------|--------|
| BBB model | BeagleBone Black (AM335x) | `config-5.10.168-ti-r83` has AM335x symbols |
| Kernel | 5.10.168-ti-r83 | `boot/uEnv.txt:3` |
| Python | 3.11.2 | `env/pyvenv.cfg` |
| DAC type | MCP4725 (12-bit, I2C) | Write protocol = MCP4725 fast write, `auto_calibrate_bno055.py` imports `adafruit_mcp4725` |
| DAC addresses | L=0x61, R=0x60 | `config.py:54-55` |
| I2C bus | Bus 2 | `motor_controller.py:32` |
| IMU | BNO055 | `main.py:150-158`, `auto_calibrate_bno055.py` |
| Encoder | YT06-OP-600B, 600 PPR | `main.py:65`, `encoder_monitor.py:27` |
| eQEP pinout | Left=P8_35/33, Right=P8_12/11 | `encoder_monitor.py:33-40`, `startup.sh` overlays |
| UART5 pins | P8_37/38 | `startup.sh` overlay reference |
| GPIO pins | P8_14=26, P8_7=66, P8_8=67, P8_9=69, P8_10=68 | `startup.sh:42` |
| Wheel radius | 0.175 m | `config.py:4` |
| Track width | 0.6 m | `config.py:5` |
| Gear ratio | 20:1 | `config.py:10` |
| Motor max RPM | 400 | `config.py:9` |
| FlySky protocol | iBUS | `flysky_receiver.py` |

### 9.2 Likely (Strong Evidence)

| Item | Detail | Reasoning |
|------|--------|-----------|
| DAC model | MCP4725 | Write protocol matches, `auto_calibrate_bno055.py` explicitly imports `adafruit_mcp4725` |
| Contactor type | Mechanical relay, NO (normally open) | GPIO HIGH = engaged, LOW = disengaged |
| Optocoupler type | Digital isolator (e.g., PC817) | Standard practice for GPIO-to-controller isolation |
| Level shifter | Analog op-amp or voltage divider | DAC output (0-5V) needs buffering for motor controller input |
| Encoder mounting | Motor shaft (pre-gearbox) | `main.py:65` comment and RPM calc uses counts/rev without gear ratio |

### 9.3 Unknown (Not Determined From Code)

| Item | Notes |
|------|-------|
| BLDC controller brand/model | "E-scooter controller" — exact specs unknown |
| Optocoupler circuit schematic | Exact resistor values, wiring unknown |
| Level shifter circuit | Gain, offset, power supply unknown |
| Contactor coil voltage | 5V? 12V? Driven by GPIO + transistor? |
| BNO055 I2C address | 0x28 is standard, not explicitly set in code |
| Power supply voltages | Battery voltage, regulator outputs unknown |
| Motor controller min throttle | At what DAC value does the motor start moving? |
| Encoder wiring | Pull-up resistors? Differential or single-ended? |
| FlySky receiver wiring | Direct UART or inverter needed? (iBUS needs inverted signal) |
| Current rating of system | Fuse values, cable gauge unknown |
| Safety circuit (E-stop) | Hardware E-stop or only software? |

---