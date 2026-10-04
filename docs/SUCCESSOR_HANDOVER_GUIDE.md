# Agricultural Ackermann Autonomous Rover: Technical Handover & Engineering Guide
**Department of Electrical Engineering, Indian Institute of Technology Palakkad**  
**Author / Outgoing Engineer:** Mr. Jagan J S (*Embedded Engineering Intern*)  
**Target Audience:** Successor Embedded Systems & Robotics Engineer  
**Date of Handover:** September 2026  

---

## Table of Contents
1. [Executive Summary & System Architecture](#1-executive-summary--system-architecture)
2. [Repository & Codebase Directory Map](#2-repository--codebase-directory-map)
3. [Hardware Specifications & Pinout Matrix](#3-hardware-specifications--pinout-matrix)
4. [STM32 Bare-Metal Firmware (`Rover_closed_loop`)](#4-stm32-bare-metal-firmware-rover_closed_loop)
5. [Raspberry Pi 5 Autonomy Stack (`RPi_companion`)](#5-raspberry-pi-5-autonomy-stack-rpi_companion)
6. [Cellular Fleet Tracker & API Stack](#6-cellular-fleet-tracker--api-stack)
7. [Calibration, Tuning & Startup Runbooks](#7-calibration-tuning--startup-runbooks)
8. [Critical Solved Pitfalls & Troubleshooting Matrix](#8-critical-solved-pitfalls--troubleshooting-matrix)
9. [Strategic Engineering Roadmap (Next Priorities)](#9-strategic-engineering-roadmap-next-priorities)

---

## 1. Executive Summary & System Architecture

### 1.1 The Platform Transformation
This project upgraded an agricultural rover from an obsolete **BeagleBone Black open-loop differential-drive** architecture to a robust, distributed **STM32F411 + Raspberry Pi 5 Ackermann steering** architecture.

```
       +-------------------------------------------------------------------+
       |                       POWER SYSTEM (54.6V)                        |
       |  54.6V Battery --> Isolator --> 100A Contactor --> 48V DC Bus     |
       |                                      |                            |
       |                   +------------------+-------------------+        |
       |                   |                                      |        |
       |           48V -> 12V Step-Down                   48V -> 5V Step   |
       |           (Actuator & Driver)                    (Logic & RPi 5)  |
       +-------------------------------------------------------------------+
                                      |
                                      v
       +-------------------------------------------------------------------+
       |                    HIGH-LEVEL COMPUTE & AUTONOMY                  |
       |                                                                   |
       |   +---------------------+             +-----------------------+   |
       |   |  Raspberry Pi 5     |<-- I2C1 ----| BNO055 9-DOF IMU      |   |
       |   |  Autonomy Stack     |             +-----------------------+   |
       |   |  - Guidance / PID   |                                         |
       |   |  - Row-Skip Planner |             +-----------------------+   |
       |   |  - Odom Fusion      |             | 4G Tracker (ESP32)    |   |
       |   +---------------------+             | - Quectel EC200U LTE  |   |
       |              |                        | - Standalone AntiTheft|   |
       |              | UART 115200 (USART2)   +-----------------------+   |
       |              v                                                    |
       +-------------------------------------------------------------------+
                                      |
                                      v
       +-------------------------------------------------------------------+
       |                 LOW-LEVEL REAL-TIME EMBEDDED CORE                 |
       |                                                                   |
       |   +-----------------------------------------------------------+   |
       |   |                 STM32F411CEU6 ("Black Pill")              |   |
       |   |  - 20 Hz Deterministic Loop (TIM5)                        |   |
       |   |  - Closed-Loop PI Wheel Velocity (wheel_pid.c)            |   |
       |   |  - Zoned Linear Actuator Steering (actuator.c)            |   |
       |   |  - Electronic Differential Math                           |   |
       |   |  - Contactor MOSFET Switch & Fault Recovery               |   |
       |   +-----------------------------------------------------------+   |
       |         |          |               |               |              |
       +---------|----------|---------------|---------------|--------------+
                 |          |               |               |
     PWM/DIR (PA8/5)    I2C1 (0x48)     I2C1 (0x60/61)  TIM2/3 Quadrature
                 |          |               |               |
                 v          v               v               v
       +-------------+ +----------+ +---------------+ +------------------+
       | Cytron MD10C| | ADS1115  | | Dual MCP4725  | | YT06-OP-1M Optical|
       | Motor Driver| | 16b ADC  | | 12-bit DACs   | | Wheel Encoders   |
       +-------------+ +----------+ +---------------+ +------------------+
              |              ^              |                 ^
              v              |              v                 |
       +-------------+ +----------+ +---------------+         |
       | PA-12 Linear| | Dual 10k | | Dual BLDC     |---------+
       | Actuator    | | Kingpin  | | Motor Drivers |
       | (Steering)  | | Pots     | | (48V 32A)     |
       +-------------+ +----------+ +---------------+
                                            |
                                            v
                                     +---------------+
                                     | Dual 750W BLDC|
                                     | 20:1 Gearbox  |
                                     +---------------+
```

### 1.2 Core Architectural Philosophy
- **Hard Real-Time Local Safety (STM32):** The STM32 bare-metal firmware runs at 20 Hz. It controls the main contactor, monitors RC failsafes, executes linear actuator positioning, and handles closed-loop PI motor velocity control. If the Raspberry Pi crashes or loses serial communication, the STM32 automatically defaults to safe idle or manual RC control within 500 ms.
- **High-Level Autonomy (Raspberry Pi 5):** The Raspberry Pi handles complex geometric trajectory generation, sensor fusion (IMU + wheel odometry), and path tracking.
- **Autonomous Sub-System Separation:** The IoT cellular security tracker runs completely independently on an ESP32 to guarantee remote telemetry and anti-theft monitoring even if the primary traction system is powered down.

---

## 2. Repository & Codebase Directory Map

All project directories reside in `/home/tuf/Documents/REPORT/`:

```
/home/tuf/Documents/REPORT/
├── Autonomous_Ackermann/
│   └── Rover_closed_loop/       <-- [CRITICAL] Active STM32 Bare-Metal Production Firmware
│       ├── Core/
│       │   ├── Inc/              <-- Headers (main.h, wheel_pid.h, stm32f4xx_it.h)
│       │   └── Src/              <-- Source files (main.c, wheel_pid.c, system_stm32f4xx.c)
│       ├── Drivers/              <-- STM32 HAL and CMSIS Drivers
│       └── Rover_closed_loop.ioc <-- STM32CubeMX Hardware Configuration File
├── RPi_companion/                <-- [CRITICAL] Raspberry Pi 5 Python Autonomy Stack
│   ├── ackermann_controller.py   <-- Path-tracking algorithms (Stanley / Pure Pursuit)
│   ├── make_straight_path.py     <-- Linear benchmarking trajectory generator
│   ├── make_rectangle_path.py    <-- Closed rectangular benchmarking generator
│   ├── make_circle_path.py       <-- Curvature benchmark generator
│   ├── make_lawnmower_path.py    <-- Full boustrophedon coverage path generator
│   ├── make_skip_row_path.py     <-- Headland skip-row agricultural path generator
│   └── rpi_stm32_bridge.py       <-- High-speed binary UART serial interface
├── STM/
│   └── Rover/                    <-- [CRITICAL] Complete KiCad v6 Hardware Schematics & PCB
│       ├── Rover.kicad_pro       <-- KiCad Project File
│       ├── Rover.kicad_sch       <-- System Schematic
│       └── Rover.kicad_pcb       <-- 2-Layer PCB Board Layout
├── UGV_Tracker/                  <-- Standalone Cellular Fleet Tracker Stack
│   ├── firmware_esp32_ec200u/    <-- ESP32 + Quectel EC200U 4G LTE/GNSS Arduino/C++ Firmware
│   ├── backend_server/           <-- FastAPI REST Server & SQLite Database (ugv_tracker.db)
│   └── web_dashboard/            <-- Dark-mode Leaflet.js Real-Time Fleet Map
├── User_API_test/
│   └── rover_api/                <-- Commercial Agricultural REST API & Simulator
├── For_future/
│   └── ROVER_RESEARCH (1).txt    <-- Outgoing research roadmap for RTK-GNSS & e-PTO
├── old_documents/                <-- Legacy BeagleBone Black schematics & flowcharts
├── internship_report.tex         <-- Complete Academic LaTeX Internship Report
└── SUCCESSOR_HANDOVER_GUIDE.md   <-- THIS DOCUMENT
```

---

## 3. Hardware Specifications & Pinout Matrix

### 3.1 Kinematic & Mechanical Parameters
| Parameter | Description | Value | Firmware / Script Reference |
| :--- | :--- | :--- | :--- |
| $L$ | Wheelbase (Front to Rear Axle) | $0.850\text{ m}$ | `ROVER_WHEELBASE_M` |
| $W$ | Track Width (Rear Wheel Stance) | $0.800\text{ m}$ | `ROVER_TRACK_WIDTH_M` |
| $r$ | Drive Wheel Outer Radius | $0.175\text{ m}$ | `ROVER_WHEEL_RADIUS_M` |
| $G$ | Planetary Gearbox Reduction | $20:1$ | Motor Output to Axle |
| $\text{RPM}_{\text{max, wheel}}$ | Maximum Rear Wheel Speed | $25\text{ RPM}$ | Equivalent to 500 RPM motor speed |
| $v_{\text{max}}$ | Maximum Forward Linear Speed | $0.458\text{ m/s}$ ($1.65\text{ km/h}$) | `MAX_LINEAR_VELOCITY` |
| $v_{\text{cruise}}$ | Autonomous Nominal Cruise Speed | $0.240\text{ m/s}$ ($0.86\text{ km/h}$) | Path tracking default |
| $\delta_{\text{max}}$ | Maximum Steering Angle | $\pm 45.0^\circ$ | `STEER_MAX_DEG` |
| $R_{\text{min}}$ | Minimum Turning Radius | $\approx 2.50\text{ m}$ | Non-holonomic steering limit |

---

### 3.2 STM32F411CEU6 Master Pinout Matrix
| Pin Name | Peripheral Function | Hardware Connection | Purpose / Electrical Spec |
| :--- | :--- | :--- | :--- |
| **`PA0`** | `TIM2_CH1` | Optical Encoder Left (CH A) | 32-bit Quadrature Input, 100nF RC filter |
| **`PA1`** | `TIM2_CH2` | Optical Encoder Left (CH B) | 32-bit Quadrature Input, 100nF RC filter |
| **`PA6`** | `TIM3_CH1` | Optical Encoder Right (CH A)| 16-bit Quadrature Input, 100nF RC filter |
| **`PA7`** | `TIM3_CH2` | Optical Encoder Right (CH B)| 16-bit Quadrature Input, 100nF RC filter |
| **`PA8`** | `TIM1_CH1` | Cytron MD10C `PWM` | Linear Actuator Speed (20 kHz PWM) |
| **`PA5`** | `GPIO_Output` | Cytron MD10C `DIR` | Linear Actuator Direction (HIGH/LOW) |
| **`PB8`** | `I2C1_SCL` | I2C Clock Bus | SCL for ADS1115 (`0x48`) & MCP4725 (`0x60/61`) |
| **`PB9`** | `I2C1_SDA` | I2C Data Bus | SDA for ADS1115 (`0x48`) & MCP4725 (`0x60/61`) |
| **`PA10`**| `USART1_RX` | FlySky FS-iA10B `iBUS` | 115200 baud Serial RX (DMA + Idle Line) |
| **`PA9`** | `USART1_TX` | Debug / Sniffer TX | Telemetry output for external debuggers |
| **`PA2`** | `USART2_TX` | Raspberry Pi 5 RX | Binary Command Link (115200 baud, DMA) |
| **`PA3`** | `USART2_RX` | Raspberry Pi 5 TX | Binary Feedback Link (115200 baud, DMA) |
| **`PB0`** | `GPIO_Output` | Contactor Gate MOSFET | Controls 48V 100A DC Main Power Contactor |
| **`PC13`**| `GPIO_Output` | Onboard Blue LED | 1 Hz Heartbeat / Error Blink |

---

## 4. STM32 Bare-Metal Firmware (`Rover_closed_loop`)

### 4.1 Execution Flow & Timing
The core loop executes in [`Autonomous_Ackermann/Rover_closed_loop/Core/Src/main.c`](file:///home/tuf/Documents/REPORT/Autonomous_Ackermann/Rover_closed_loop/Core/Src/main.c):
1. **Hardware Initialization (`0.0 - 0.2s`):**
   - Configures system clocks (100 MHz PLL from 25 MHz HSE).
   - Initializes GPIOs and immediately drives `PB0 = HIGH` to close the main 48V contactor.
   - Starts timers: `TIM1` (Actuator PWM), `TIM2` (Left Encoder), `TIM3` (Right Encoder), `TIM5` (20 Hz loop tick).
   - Initializes `I2C1` and tests connectivity to ADS1115 (`0x48`), Left DAC (`0x60`), and Right DAC (`0x61`).
   - Starts DMA reception on `USART1` (iBUS) and `USART2` (RPi5 link).
2. **20 Hz Control Iteration (Every 50 ms):**
   - **Step 1: Read Feedback:** Reads kingpin potentiometer angles via ADS1115 and updates wheel RPMs from `TIM2`/`TIM3` encoder count deltas.
   - **Step 2: Parse Control Inputs:**
     - Checks iBUS Channel 5 (Autonomous Mode Switch).
     - If `CH5 > 1500`: Mode is **Autonomous** $\rightarrow$ parse target speed and steering angle from Raspberry Pi USART2 packet.
     - If `CH5 <= 1500` or failsafe triggered: Mode is **Manual RC** $\rightarrow$ parse speed from Channel 2 (Throttle) and steering from Channel 1 (Aileron).
   - **Step 3: Steering Control (`actuator.c`):**
     - Compares target steer angle $\delta_{\text{target}}$ against measured angle $\delta_{\text{actual}}$.
     - Calculates linear actuator PWM and direction using the **Dual-Zone Control Algorithm**.
   - **Step 4: Wheel Velocity PI Control (`wheel_pid.c`):**
     - Calculates required individual wheel linear speeds using Ackermann differential geometry:
       $$v_L = v \left(1 - \frac{W}{2L} \tan\delta\right), \quad v_R = v \left(1 + \frac{W}{2L} \tan\delta\right)$$
     - Converts velocities to target RPMs and executes discrete PI velocity control loop.
     - Writes resulting 12-bit values (0–4095) to Left and Right MCP4725 DACs.
   - **Step 5: Telemetry Transmit:** Packages current rover state (Actual $\delta$, Left/Right RPM, Battery Voltage, Mode) and sends via USART2 DMA to Raspberry Pi 5.

---

### 4.2 Closed-Loop PI Wheel Velocity Controller (`wheel_pid.c`)
To overcome planetary gearbox static friction (stiction) and maintain constant speed under load:

```c
// Closed-loop PI parameters (Tuned for 750W BLDC + 20:1 Gearbox)
#define WHEEL_PID_KP            50.0f
#define WHEEL_PID_KI            15.0f
#define THR_FWD_MIN_DAC         1624     // Feedforward stiction overcoming floor (2.0V)
#define THR_REV_MIN_DAC         1624
#define DAC_MAX_LIMIT           4000     // 4.88V safe upper clamp
#define INTEGRAL_WINDUP_LIMIT   800.0f

void Wheel_PID_Compute(WheelPID_t *pid, float target_rpm, float measured_rpm, uint16_t *dac_out) {
    if (target_rpm == 0.0f) {
        pid->integral = 0.0f;
        *dac_out = 0;
        return;
    }
    
    float error = target_rpm - measured_rpm;
    pid->integral += error * 0.050f; // dt = 50ms (20Hz)
    
    // Anti-windup clamping
    if (pid->integral > INTEGRAL_WINDUP_LIMIT) pid->integral = INTEGRAL_WINDUP_LIMIT;
    if (pid->integral < -INTEGRAL_WINDUP_LIMIT) pid->integral = -INTEGRAL_WINDUP_LIMIT;
    
    float control_val = (WHEEL_PID_KP * error) + (WHEEL_PID_KI * pid->integral);
    
    // Combine feedforward friction floor + PI correction
    int32_t output = THR_FWD_MIN_DAC + (int32_t)control_val;
    if (output < THR_FWD_MIN_DAC) output = THR_FWD_MIN_DAC;
    if (output > DAC_MAX_LIMIT)   output = DAC_MAX_LIMIT;
    
    *dac_out = (uint16_t)output;
}
```

---

### 4.3 Dual-Zone Steering Control Law (`actuator.c`)
To avoid linear actuator limit-cycle oscillation around the mechanical deadband:
- **Zone 1 ($|\Delta\delta| > 4.0^\circ$):** Bang-bang maximum duty ($100\%$) for high slew rate.
- **Zone 2 ($1.5^\circ < |\Delta\delta| \le 4.0^\circ$):** Scaled proportional duty ($40\% - 75\%$).
- **Deadband ($|\Delta\delta| \le 1.5^\circ$):** Duty set strictly to $0\%$ (`ACT_MIN_DUTY_PCT = 0.0f`).

---

## 5. Raspberry Pi 5 Autonomy Stack (`RPi_companion`)

### 5.1 Software Setup & Environment
The Raspberry Pi 5 runs Raspberry Pi OS (64-bit). The companion stack resides in `/home/tuf/Documents/REPORT/RPi_companion`.

To prepare the Python virtual environment:
```bash
cd /home/tuf/Documents/REPORT/RPi_companion
python3 -m venv venv
source venv/bin/activate
pip install numpy scipy pyserial smbus2
```

---

### 5.2 Serial Protocol Specification (STM32 $\longleftrightarrow$ RPi 5)
Communication runs on `USART2` (`/dev/ttyAMA0` on RPi 5) at **115200 baud, 8N1**.

#### A. Command Packet (RPi 5 $\rightarrow$ STM32) — Sent at 20 Hz:
| Byte Index | Field | Type | Description |
| :--- | :--- | :--- | :--- |
| `0` | Header 1 | `uint8_t` | `0xAA` |
| `1` | Header 2 | `uint8_t` | `0x55` |
| `2` | Packet Type | `uint8_t` | `0x01` (Velocity & Steering Command) |
| `3-4` | Target Speed ($v$) | `int16_t` | Speed in $\text{mm/s}$ (e.g., $240 = 0.24\text{ m/s}$) |
| `5-6` | Target Steer ($\delta$)| `int16_t` | Angle in centidegrees (e.g., $1520 = +15.20^\circ$) |
| `7` | Autonomous Enable | `uint8_t` | `1` = Autonomous control active, `0` = Standby |
| `8` | Checksum | `uint8_t` | Bitwise XOR of Bytes 2 through 7 |

#### B. Telemetry Packet (STM32 $\rightarrow$ RPi 5) — Received at 20 Hz:
| Byte Index | Field | Type | Description |
| :--- | :--- | :--- | :--- |
| `0` | Header 1 | `uint8_t` | `0xAA` |
| `1` | Header 2 | `uint8_t` | `0x55` |
| `2` | Packet Type | `uint8_t` | `0x81` (Rover State Telemetry) |
| `3-4` | Measured Steer ($\delta$) | `int16_t` | Kingpin angle in centidegrees |
| `5-6` | Left Wheel Speed | `int16_t` | Measured Left RPM $\times 10$ |
| `7-8` | Right Wheel Speed| `int16_t` | Measured Right RPM $\times 10$ |
| `9-10`| Battery Voltage | `uint16_t`| Battery bus voltage in mV ($54600 = 54.6\text{V}$) |
| `11` | Rover State Flags| `uint8_t` | Bit 0: Contactor closed, Bit 1: Auto mode, Bit 2: Failsafe |
| `12` | Checksum | `uint8_t` | Bitwise XOR of Bytes 2 through 11 |

---

### 5.3 Agricultural Trajectory Generators & Execution
All path scripts generate standard waypoint CSV files formatted as `[x_m, y_m, speed_mps, target_steer_deg]`:

1. **Straight Benchmark Run:**
   ```bash
   python make_straight_path.py --length 20.0 --speed 0.25 --output straight_20m.csv
   python ackermann_controller.py --path straight_20m.csv --port /dev/ttyAMA0
   ```
2. **Headland Skip-Row Agricultural Coverage:**
   Generates smooth, non-holonomic turns between non-adjacent crop rows (Row $1 \rightarrow 3 \rightarrow 5 \rightarrow 2 \rightarrow 4$):
   ```bash
   python make_skip_row_path.py --rows 6 --row_len 30.0 --row_spacing 1.2 --turn_radius 2.6 --output skip_row_field.csv
   python ackermann_controller.py --path skip_row_field.csv --port /dev/ttyAMA0
   ```

---

## 6. Cellular Fleet Tracker & API Stack

### 6.1 ESP32 Tracker Architecture (`UGV_Tracker`)
The tracker operates independently on a custom ESP32 board paired with a **Quectel EC200U 4G LTE/GNSS module**.
- **Location:** [`UGV_Tracker/firmware_esp32_ec200u/`](file:///home/tuf/Documents/REPORT/UGV_Tracker/firmware_esp32_ec200u)
- **Operation:** Transmits JSON telemetry via HTTP POST over cellular 4G every 5 seconds to `/api/v1/telemetry`.

### 6.2 Starting the Cloud Backend & Web Dashboard
```bash
cd /home/tuf/Documents/REPORT/UGV_Tracker/backend_server
# Start FastAPI backend on port 8000
python3 -m venv venv && source venv/bin/activate
pip install fastapi uvicorn sqlite3 pydantic
uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```
Open a browser to `http://<server-ip>:8000/dashboard` to view the Leaflet real-time dark-mode map.

---

## 7. Calibration, Tuning & Startup Runbooks

### 7.1 Front Steering Potentiometer Calibration Procedure
Before operating the rover, calibrate the front kingpin angle feedback:
1. **Mechanical Centering:**
   - Power off the traction system.
   - Manually adjust the steering linkage until both front wheels are exactly parallel to the chassis centerline ($0.0^\circ$).
2. **ADC Zero Reading:**
   - Read the ADS1115 single-ended channel `AIN0` voltage via I2C.
   - Note the center voltage $V_{\text{center}}$ (nominally $\approx 2.50\text{V}$, raw $\text{ADC} \approx 13200$).
3. **Full Lock Verification:**
   - Turn steering to full Left Lock ($+45.0^\circ$): Record $V_{\text{left}}$ ($\approx 4.10\text{V}$).
   - Turn steering to full Right Lock ($-45.0^\circ$): Record $V_{\text{right}}$ ($\approx 0.90\text{V}$).
4. **Update Scale Constants in `actuator.h`:**
   ```c
   #define STEER_POT_CENTER_V      2.500f
   #define STEER_POT_VOLTS_PER_DEG 0.0355f // (V_left - V_right) / 90.0 deg
   ```

---

### 7.2 Field Power-Up Sequence (Checklist)
1. **Physical Pre-Flight Check:**
   - Verify tire pressures (28 PSI).
   - Check steering linkage tie-rods and kingpin bearings for mechanical play.
   - Confirm optical encoder disc alignment (clear of dirt and debris).
2. **Electrical Power-On:**
   - Turn the **Manual Battery Isolator Switch** to ON.
   - Power on the FlySky RC Transmitter. Verify Channel 5 (Auto switch) is in the **DOWN (MANUAL)** position.
   - Turn on the logic power switch.
   - Observe the STM32 Blue LED (`PC13`):
     - Solid for 0.5s $\rightarrow$ Blinking steadily at 1 Hz indicates normal operation.
     - You will hear an audible `CLICK` as the 100A main DC contactor energizes.
3. **Manual RC Verification:**
   - Gently push RC Throttle: Both rear wheels must turn forward synchronously.
   - Move Steering stick Left/Right: Linear actuator must articulate front wheels smoothly without buzzing or limit-cycling.
4. **Autonomous Mode Handover:**
   - Boot Raspberry Pi 5 and launch `ackermann_controller.py`.
   - Confirm serial handshake: The terminal will report `[STM32 LINK] Connected | Batt: 54.2V | Steer: 0.1 deg`.
   - Flip FlySky Channel 5 to **UP (AUTO)**. The rover will begin autonomous waypoint tracking.
   - **Emergency Stop:** Flipping Channel 5 DOWN immediately returns control to manual RC; shutting off the transmitter triggers hardware failsafe neutral within 100 ms.

---

## 8. Critical Solved Pitfalls & Troubleshooting Matrix

| Symptom / Failure Mode | Root Cause Identified | Engineering Solution Implemented |
| :--- | :--- | :--- |
| **System freezes on cold boot; DACs unresponsive** | `Motor_Init()` tried to query I2C DACs before the 48V contactor closed, leaving DACs unpowered on an unpowered sub-rail. | Contactor switch (`PB0`) is now energized **unconditionally first**. Added `_RecoverI2cBus()` (9 clock pulses on `PB8`) before I2C initialization. |
| **Front steering oscillates violently around $0^\circ$** | Minimum PWM floor (`ACT_MIN_DUTY_PCT`) was set to $70\%$, causing the linear actuator to overshoot the deadband continuously. | Set `ACT_MIN_DUTY_PCT = 0.0f` and implemented dual-zone control (100% duty $>4^\circ$, PID $\le 4^\circ$, $0\%$ within $1.5^\circ$). |
| **Rover stalls at low speeds or surges unpredictably** | BLDC motor controller deadband and 20:1 planetary gearbox static friction (stiction). | Implemented closed-loop PI speed control (`wheel_pid.c`) with a feedforward friction overcoming baseline (`THR_FWD_MIN_DAC = 1624`). |
| **Encoder RPM jumps randomly in electrical noise** | Motor driver switching noise induced spurious voltage spikes on high-impedance optical encoder wires. | Added $100\text{ nF}$ ceramic low-pass filter capacitors between encoder signal lines (A/B) and GND directly at the STM32 screw terminals. |
| **Loss of Raspberry Pi serial packets under CPU load** | OS kernel context switches delayed standard user-space `read()` calls on UART. | Implemented STM32 USART2 DMA circular ring-buffer reception with Idle Line Interrupt detection (`IDLEIE`). |

---

## 9. Strategic Engineering Roadmap (Next Priorities)

Based on field trials and the research notes in [`For_future/ROVER_RESEARCH (1).txt`](file:///home/tuf/Documents/REPORT/For_future/ROVER_RESEARCH%20%281%29.txt), the recommended next engineering phases are:

1. **Centimeter-Level RTK GNSS + EKF Fusion:**
   - Interface a dual-frequency RTK GNSS receiver (e.g., u-blox ZED-F9P) over UART to the Raspberry Pi 5.
   - Implement an Extended Kalman Filter (EKF) using `robot_localization` to fuse RTK GNSS positions with wheel encoder odometry and BNO055 IMU heading.
2. **Electronic Power Take-Off (e-PTO) Implementation:**
   - Design an auxiliary 48V / 12V high-power solid-state relay switching board controlled by spare STM32 pins (`PB12`, `PB13`).
   - Enables companion-commanded power to rotary weeders, seeders, and pesticide spray booms.
3. **Perception & Crop-Row Visual Servoing:**
   - Mount an Intel RealSense D435i or Luxonis OAK-D stereo camera on the front mast.
   - Implement an OpenCV / YOLO-based crop-line detection pipeline to dynamically center the rover between plant canopies without relying solely on GPS coordinates.

---
*For any hardware questions or schematic clarifications, refer to the KiCad files in `STM/Rover` and the complete academic thesis in `internship_report.tex`.*
