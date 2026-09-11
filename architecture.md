# Architecture Plan & System Design — Closed-Loop Ackermann Steering

## 1. System Overview & Platform Context
The Agriculture Rover is a robotic platform powered by the **STM32F411CEU6 (Black Pill)** microcontroller (running bare-metal C with STM32Cube HAL at 100 MHz).

The closed-loop Ackermann version operates in the **`Rover_closed_loop/`** project directory, leaving the original open-loop `Rover/` codebase untouched as a verified fallback.

### Hardware Interconnects & Pin Allocation
| Pin | Function | Peripheral / Mode | Purpose |
|---|---|---|---|
| **PA10** | iBUS RX | USART1_RX (DMA2 Stream 2) | FlySky FS-iA6B RC receiver stream (115200 baud) |
| **PA9** | Telemetry TX | USART1_TX (Direct register DR) | ESP32 telemetry bridge (115200 baud) |
| **PA8** | Actuator PWM | TIM1_CH1 (PWM Generation) | MD10C speed input (0–100% duty) |
| **PA5** | Actuator DIR | GPIO Output (`Actuator_DIR`) | MD10C direction input (HIGH=Extend/Left, LOW=Retract/Right) |
| **PB6** | I2C1 SCL | I2C1 (100 kHz Standard Mode) | Shared bus: MCP4725 DACs (0x60, 0x61) + ADS1115 ADC (0x48) |
| **PB7** | I2C1 SDA | I2C1 (100 kHz Standard Mode) | Shared bus: MCP4725 DACs (0x60, 0x61) + ADS1115 ADC (0x48) |
| **PB0** | Contactor | GPIO Output PP | Main motor controller power relay |
| **PB1** | Rev Left | GPIO Output PP | Left motor controller reverse direction |
| **PB10** | Brake Left | GPIO Output PP | Left motor controller brake |
| **PB12** | Brake Right | GPIO Output PP | Right motor controller brake |
| **PB13** | Rev Right | GPIO Output PP | Right motor controller reverse direction |
| **PA0 / PA1** | Encoder L | TIM2 (32-bit, TI12 Mode) | Left wheel shaft optical encoder |
| **PA6 / PA7** | Encoder R | TIM3 (16-bit, TI12 Mode) | Right wheel shaft optical encoder |
| **PC13** | Status LED | GPIO Output PP (Active-LOW) | Onboard diagnostic fault indicator |
| **PA2** | RPi TX | USART2_TX | → RPi5 Pin10 / GPIO15 (RXD) — command/feedback link, firmware flashed, RPi side not yet wired/tested |
| **PA3** | RPi RX | USART2_RX | ← RPi5 Pin8 / GPIO14 (TXD) — command/feedback link, firmware flashed, RPi side not yet wired/tested |

---

## 2. Decoupled Closed-Loop Control Architecture

The system decouples steering from propulsion to prevent tire scrubbing and guarantee kinematic accuracy:

```
[RC Joystick X] ──→ Target Steering Angle δ_target (−45° to +45°)
                             │
                             ▼
[Steering Loop] ────→ Steer PID (Derivative-on-Measurement) ──→ MD10C (PA8/PA5) ──→ Linear Actuator
                             ▲                                                              │
                             │                                                              ▼
                      Measured Angle δ_actual ◄── ADS1115 (0x48, AIN0/AIN1) ◄── 2× 10k Potentiometers
                             │
                             ▼
[Propulsion Loop] ──→ Ackermann Electronic Differential (tan(δ)) ──→ Rear Wheel DACs (Left/Right)
                             ▲
                             │
[RC Joystick Y] ───→ Base Speed V_base
```

### 2.1 Steering Feedback Loop (`ads1115.c` & `actuator.c`)
- **ADS1115 ADC:** Reads Left Wheel Angle (`AIN0`) and Right Wheel Angle (`AIN1`) over I2C1 at 860 SPS (~1.16 ms conversion).
- **Fault Debounce:** Uses `STEER_FAULT_THRESHOLD = 5` consecutive bad reads before declaring a sensor failure; transient 1-tick glitches maintain the previous angle without stalling the actuator.
- **Actuator Full-Speed Directional Control:**
  - Because the linear actuator moves at ~7 mm/s max speed, it runs at **100% full speed** whenever outside the deadband ($\pm 0.5^\circ$), eliminating unnecessary speed throttling.
  - When within $\pm 0.5^\circ$ of target angle, PWM stops (`0%`) and the linear actuator mechanically holds position.

### 2.2 Propulsion & Differential Loop (`ackermann.c`)
- **Kinematic Radius:** Uses exact bicycle model turning radius about the rear axle center:
  $$R_{\text{rear}} = \frac{W_b}{\tan(\delta)}$$
- **Wheel Speeds:**
  $$V_{\text{right}} = V_{\text{base}} \left(1 + \frac{L_t}{2 R_{\text{rear}}}\right), \quad V_{\text{left}} = V_{\text{base}} \left(1 - \frac{L_t}{2 R_{\text{rear}}}\right)$$
- **Standstill Pivot Mode:** Full steering deflection withholds drive until wheels swing past 80% of target lock ($36^\circ$), then locks the inner rear wheel at 0 RPM while the outer rear wheel crawls forward at ~7.5 RPM to pivot around the inner tire.

---

## 3. Autonomous Integration — Raspberry Pi Companion Computer (Planned)

**Goal:** restore the autonomous waypoint-following capability the platform had on the BeagleBone Black (see `Documentations/Rover_study.md` §13 and `Old_files/UGV_closed/`), but re-architected around the STM32 as a real-time actuation server rather than the BBB's single-process "brain."

### 3.1 Division of Responsibility

| Layer | Runs on | Responsibility |
|---|---|---|
| **Guidance & sensing** | Raspberry Pi | IMU + RTK GNSS fusion, localization (UTM conversion), path/waypoint management, guidance law (P/ISMC → target `V`, `δ`) |
| **Real-time actuation** | STM32F411 | Steering PID/bang-bang + ADS1115 feedback, Ackermann electronic differential, encoder feedback, contactor/arming, all existing RC failsafes |

This is a deliberate inversion of the old BBB architecture, where the BBB itself ran guidance *and* drove the DACs/GPIO directly, and the RPi was only a passive RTK data source over TCP (`Old_files/UGV_closed/rtk_receiver.py`). Here, the RPi becomes the guidance brain; the STM32 keeps owning every safety-critical/real-time function it already owns today, and simply gains a second command source (UART from the RPi) alongside the existing RC iBUS source.

### 3.2 Physical Link

STM32 **USART2** (PA2 TX / PA3 RX) ↔ Raspberry Pi 5 primary UART (`GPIO14`/Pin8 TXD, `GPIO15`/Pin10 RXD), GND common. Configured in `Rover.ioc` and generated — see §3.6.

Cross-connect (TX→RX both ways):
| STM32 | RPi5 |
|---|---|
| PA3 (USART2_RX) | Pin8 / GPIO14 (TXD) |
| PA2 (USART2_TX) | Pin10 / GPIO15 (RXD) |
| GND | GND (any GND pin) |

**✅ RPi5 setup complete and verified working** (link confirmed end-to-end, 20 Hz, zero corruption). Three things were needed, not just the console — see §3.6/§3.8 for the full story: (1) disable the login console on `GPIO14`/`GPIO15`, (2) disable onboard Bluetooth (`dtoverlay=disable-bt`, it silently claims the same UART), (3) `enable_uart=1` (the actual root cause of an extended debugging session — without it the pins are never muxed to UART at the hardware level at all).

### 3.3 Mode Arbitration

- **SWD** (unchanged): arms/disarms the contactor, exactly as today.
- **SWB** (repurposed): was the 2-position reverse-speed-cap switch (`SWB_SPEED_LOW/HIGH` = 60%/100%); becomes a pure **MANUAL / AUTO** command-source select. Reverse-speed-cap feature is dropped — reverse runs at the same cap as forward once this lands (see scratchpad.md).
- When armed **and** SWB = MANUAL → command source is RC iBUS (`Xn`, `Yn`), as today.
- When armed **and** SWB = AUTO → command source is the UART link from the RPi (target steering angle `δ_target`, target speed `V_target`), fed into the same downstream steering PID / Ackermann differential the RC path already uses.

### 3.4 UART Protocol (implemented in firmware, not yet hardware-tested)

20 Hz, binary, fixed-size frames, matching the main loop's own rate.

**Command frame — RPi → STM32:**
| Field | Type | Meaning |
|---|---|---|
| header | `0xAA 0x55` | frame sync |
| steer_target | `int16` (°×100) | target steering angle, −45.00° to +45.00° |
| speed_target | `int16` (mm/s) | target linear speed, signed (+fwd / −rev) |
| seq | `uint8` | rolling counter — reserved for detecting a stalled-but-connected RPi (repeated seq = link up, RPi logic frozen); parsed but **not yet acted on** in `rpi_link.c` |
| checksum | `uint8` | XOR of all preceding bytes, same spirit as the existing iBUS checksum on PA10 |

**Feedback frame — STM32 → RPi:**
| Field | Type | Meaning |
|---|---|---|
| header | `0xBB 0x66` | frame sync |
| angle_L, angle_R | `int16` each (°×100) | measured steering angle per wheel (ADS1115) |
| rpm_L, rpm_R | `int16` each (RPM×10) | encoder wheel speed |
| status | `uint8` bitfield | armed / steer-fault / actuator-fault / contactor state |
| checksum | `uint8` | |

### 3.5 Failsafe

New `AUTO_UART_TIMEOUT_MS` (~300 ms, same order as the existing 400 ms RC timeout). If SWB = AUTO and no valid command frame arrives within that window, the STM32 forces `BRAKE` — same behavior class as RC signal loss today — regardless of the last command received. Flipping SWB back to MANUAL always hands control back to the RC sticks immediately, independent of UART link state.

**Not yet decided:** whether stick deflection should also force a manual takeover while SWB is still in AUTO (the old BBB `safety.py` did this at a 15% threshold). Left open for now — SWB is the sole arbiter until/unless this is explicitly added.

### 3.6 RPi-Side Software (`RPi_companion/`)

New top-level directory for all Raspberry Pi 5 Python code — parallel to `Rover_closed_loop/` and `Documentations/`. Started with the IMU driver; guidance/localization/UART-client modules land here as they're built.

**Hardware:** DFRobot Fermion BNO055 9-axis IMU, I2C1 on the RPi5's 40-pin header — VCC→Pin1 (3V3), SDA→Pin3 (GPIO2), SCL→Pin5 (GPIO3), GND→Pin6. Default I2C address `0x28` (ADR pin floating/low).

| File | Purpose |
|---|---|
| `RPi_companion/requirements.txt` | `adafruit-blinka`, `adafruit-circuitpython-bno055` — same library family the old BBB stack used (`Old_files/dev_bak/test_imu.py`), continued here since Blinka now supports the Pi 5 |
| `RPi_companion/imu.py` | `IMU` driver class — `board.I2C()` + `adafruit_bno055`, returns an `IMUData` sample (heading/roll/pitch deg, gyro_z rad/s, accel_x/y m/s², calibration status, `valid`). Fault handling deliberately mirrors `ads1115.c` on the STM32 side: holds the last valid sample across transient I2C glitches, only flips `valid=False` after `IMU_FAULT_THRESHOLD` (5) consecutive bad reads. Supports optional calibration-offset preload via `IMU_CALIBRATION_OFFSETS` |
| `RPi_companion/test_imu.py` | Live readout script — **run this first** after wiring, before trusting `imu.py` in anything else, to confirm hardware/wiring/address are correct |
| `RPi_companion/calibrate_imu.py` | Waits for full calibration (Sys/Gyro/Accel/Mag all = 3), then reads back and prints the sensor's internal offset registers in a form that pastes directly into `imu.py`'s `IMU_CALIBRATION_OFFSETS` |
| `RPi_companion/uart_link.py` | `STM32Link` driver for the STM32 UART link (counterpart to `rpi_link.c`) — background thread parses feedback frames (header-scan + checksum, no hardware IDLE-line equivalent on this side), exposes `read()`/`send_command()`. Frame formats hand-copied from `rpi_link.c`'s header comment; the two sides have no shared schema file, keep them in sync manually if the protocol ever changes |
| `RPi_companion/check_stm32_link.py` | Connectivity check — sends neutral (0,0) command frames, reports feedback frame rate/checksum failures/status bits. Sending is inert while SWB=MANUAL; proves the RX half (STM32→RPi) directly, the TX half only indirectly (see the script's own header comment for the caveat and why) |
| `RPi_companion/read_encoders.py` | Passive live readout of wheel RPM + steering angle from the feedback frame — sends nothing, safe to run anytime including during manual driving |
| `ESP32_uart_sniffer/ESP32_uart_sniffer.ino` | Read-only ESP32 tap on STM32 `PA2` (USART2_TX) only — isolates whether the STM32 is physically transmitting feedback frames, independent of the RPi's software/wiring entirely. Same one-way-bridge safety rule as the existing PA9 debug bridge: never connect its TX pin to `PA3` while the RPi is also wired there (two transmitters on one line). See §3.8 for the diagnostic session that motivated this. |

**Setup checklist (on the RPi5 itself):**
- [x] Disable the Linux serial console on `GPIO14`/`GPIO15` (Pin8/Pin10) — done directly (not via `raspi-config`, which wasn't present in this Debian 13 "trixie" image): `systemctl disable --now serial-getty@ttyAMA10.service` + removed `console=serial0,115200` from `/boot/firmware/cmdline.txt`. Confirmed `inactive`/`disabled` and non-regenerating after a reboot (the unit is runtime-generated from the `console=` kernel parameter, so removing that parameter is what actually makes it permanent, not the `disable` alone).
- [x] Disable onboard Bluetooth (`dtoverlay=disable-bt`) — it silently claims the same UART as GPIO14/15 at boot, independent of the console-getty fix above. **Required, not optional, for this UART to work.** Disables BT on this Pi permanently (not used elsewhere in this project).
- [x] Add `enable_uart=1` to `config.txt` — **this was the actual root cause of the "zero bytes" debugging session** (§3.8): without it, GPIO14/15 are never switched into UART alternate-function mode at the hardware level, even though `/dev/serial0` exists as a device node. Verify with `pinctrl get 14,15` → expect `GPIO14 = TXD0`, `GPIO15 = RXD0` (not `none`).
- [x] Confirm I2C1 is enabled — uncommented `dtparam=i2c_arm=on` in `/boot/firmware/config.txt`, rebooted. Confirmed live: `i2cdetect -l` shows `i2c-1` ("Synopsys DesignWare I2C adapter"). `board.I2C()` correctly auto-detects `RASPBERRY_PI_5` with `SCL=GPIO3`/`SDA=GPIO2`.
- [ ] `i2cdetect -y 1` with the sensor attached — confirm a device shows at `0x28` (or `0x29` if the ADR pin is strapped high; pass `address=0x29` to `IMU()` if so). **Blocked: IMU not physically connected this session** (Pi was on the bench, no peripherals).
- [x] `python3 -m venv ~/rover-venv && source ~/rover-venv/bin/activate && pip install -r RPi_companion/requirements.txt` — needed two extra system packages first, not obvious from the Python package alone: `sudo apt-get install -y swig liblgpio-dev` (the `lgpio` GPIO backend has no prebuilt wheel for this OS/arch and fails to compile/link without them). Documented in `requirements.txt`'s header comment.
- [x] Deployed `RPi_companion/` to `~/RPi_companion` on the Pi.
- [x] `python3 test_imu.py` with nothing attached — confirmed it fails cleanly (`No I2C device at address: 0x28`, exit 1, no hang/crash) rather than silently returning fake data. This is as far as verification can go without the sensor.
- [ ] Re-run `test_imu.py` once the BNO055 is physically reconnected — confirm heading/roll/pitch update sensibly as the board is moved, `valid=OK`.
- [ ] `python3 calibrate_imu.py` — run the still/6-orientation/figure-8 ritual, get all four calibration values to 3, record the printed offsets in scratchpad.md and paste into `imu.py`.

**Not reused from the old BBB rig:** the saved `bno055_calibration.json` offsets (`accel[0,0,0]`, `gyro[-2,0,1]`, `mag[111,-187,-126]`) — those are specific to that sensor's old physical mounting and magnetic environment, not this one. Fresh calibration required.

### 3.7 Firmware File Reference (Phase 5)

| File | Purpose |
|---|---|
| `Core/Inc/rpi_link.h`, `Core/Src/rpi_link.c` | USART2 DMA+IDLE framing, command parse, feedback send — mirrors `ibus.c`'s role for USART1 |
| `Core/Inc/ibus.h`, `Core/Src/ibus.c` | Modified: `SWB` now decodes to `IBUSData_t.auto_mode` via `_SwbToAuto()`; `rev_speed_pct` fixed at `1.0f` |
| `Core/Inc/config.h` | Modified: SWB section rewritten, `SWB_SPEED_LOW/HIGH` removed, new RPi link protocol/timeout constants added |
| `Core/Inc/ackermann_config.h` | Modified: added `AUTO_SPEED_DEADBAND_MS` |
| `Core/Inc/ackermann.h`, `Core/Src/ackermann.c` | Modified: added `Ackermann_RunAuto()` |
| `Core/Src/main.c` | Modified: reads `RPiCmd_t` each tick, branches on `rc.auto_mode`, sends feedback frame every tick; also had missing includes restored (see scratchpad.md) |
| `Core/Src/stm32f4xx_it.c` | Modified: `USART2_IRQHandler` now calls `RPiLink_IdleCallback()` on the IDLE flag |
| `Rover.ioc` | Modified: `USART2` peripheral (PA2/PA3), `DMA1_Stream5` RX request, associated NVIC entries |

**Status: RESOLVED — link fully verified working end-to-end.** Rover on a jack for bench testing; `check_stm32_link.py` shows a steady 20.0 frames/s with zero checksum failures, `read_encoders.py` shows live, sane angle/RPM data. See §3.8 for the root cause and the debugging trail that found it.

### 3.8 UART Link Debugging Session — RESOLVED

**Symptom:** first end-to-end test (`RPi_companion/check_stm32_link.py`) found zero bytes arriving at the RPi, confirmed at the raw-byte level (bypassing all app parsing).

**Ruled out along the way, by code audit + physical checks** (STM32 firmware code, `check_stm32_link.py`'s own logic, a stale `.elf`, GPIO pin conflicts, a boot-time `Error_Handler()` hang, and unbounded I2C calls in the boot path) — none of these were the cause. `ESP32_uart_sniffer/` (a read-only tap directly on STM32 `PA2`, isolated from the RPi entirely) confirmed the STM32 **was** transmitting correctly the whole time, and the user independently re-confirmed the physical wiring was correct — which narrowed the fault to the RPi's own OS/config, exactly as the user suspected.

**Root cause — two stacked problems on the RPi side:**
1. **Bluetooth was silently claiming the UART.** `dmesg` showed `hci_uart_bcm serial0-0: ...` — the Pi 5's onboard Bluetooth shares the same physical UART as GPIO14/15. Freeing the login console (§3.6's original setup) wasn't sufficient; Bluetooth's own boot-time attach process re-claimed the port independently, leaving it at 9600 baud with the HCI line discipline attached instead of a plain raw serial port. Fixed with `dtoverlay=disable-bt` in `/boot/firmware/config.txt` — **this permanently disables Bluetooth on this Pi**, an accepted trade-off since BT isn't used anywhere in this project.
2. **The real root cause: `enable_uart=1` was never set.** Even after fixing Bluetooth, zero bytes persisted. `pinctrl get 14,15` (RP1's pin-mux inspection tool) revealed `GPIO14 = none`, `GPIO15 = none` — the physical pins were never switched into UART alternate-function mode at the hardware level, despite `/dev/ttyAMA10` existing as a kernel device node and a `serial-getty` unit having appeared to run on it previously. **A device node existing, or even a getty being "active," does not prove the GPIO pins are actually muxed to the peripheral** — that assumption is what cost the most debugging time. Fixed by explicitly adding `enable_uart=1` to `config.txt`; confirmed after reboot via `pinctrl get 14,15` → `GPIO14 = TXD0`, `GPIO15 = RXD0`.

**Verification after both fixes:** raw `cat /dev/serial0` (with baud explicitly forced to 115200 via `stty`, since `cat` doesn't set it itself) showed clean, correctly-checksummed feedback frames. `check_stm32_link.py` then confirmed the full application-layer path: steady 20.0 frames/s, zero bad frames, `ARMED=Y RC_OK=Y`. `read_encoders.py` confirmed live sane data (`rpm_L`/`rpm_R` = 0.00, correct with the rover on a jack; `angle_L≈+39°` with normal jitter).

**One new observation, not yet investigated:** `angle_R` reads a rock-steady `-45.00°` with zero jitter, unlike `angle_L` which shows normal small ADC noise. Either the right wheel is genuinely sitting at full lock, or that channel is saturating/clamping. Not urgent (`STEER_FAULT` reports healthy) but worth checking against the physical wheel position next time someone's at the rover.

**Lesson for future RPi UART/GPIO work:** verify pin mux directly with `pinctrl get <pin>` before trusting a device node's existence or a systemd unit's "active" status as proof the hardware is actually configured — this applies to any future GPIO-peripheral wiring on this Pi, not just this UART.

### 3.9 Guidance Stack Design (No RTK — Dead Reckoning Only)

Studied `Old_files/dev_bak/`'s BBB-era encoder+IMU-only autonomous scripts (`ugv_p_waypoint.py`, `ugv_pid_waypoint.py`, `ugv_p_circle.py`/`_eight.py`/`_lawnmower.py`/`_lawnmower1.py`, and the 9-axis variant) before designing this — they're the closest prior art to what this rover needs, since it has no RTK wired to the RPi either. Key findings from that study, and the design decisions built on them:

**How the BBB system worked:** encoder ticks → distance traveled; BNO055 IMU's own fused Euler heading (not integrated from wheel odometry) → absolute-ish yaw. Position: `x += d·cos(yaw); y += d·sin(yaw)`. An outer P or PID loop (`ugv_p_*.py` vs `ugv_pid_*.py` — same skeleton, differ only in whether the outer loop has an I/D term) turned distance-to-goal and heading-error into linear/angular velocity, converted to differential wheel RPM, closed by an inner PI loop on encoder RPM. All of `waypoint`/`circle`/`eight`/`lawnmower` variants share one identical waypoint loader — **the pattern shape lived entirely in which CSV was loaded, not in the script.** The circle/eight/lawnmower CSVs themselves no longer exist anywhere in the backup.

**Chassis constants matched ours exactly:** `a=0.175` (wheel radius), `d=0.30` (half-track) are `WHEEL_RADIUS_M`/`TRACK_WIDTH_M/2` from our own `config.h` — but that geometry is for **differential drive**; this rover is Ackermann, so the low-level V,W→wheel-RPM conversion and the BBB's motor/DAC/PI-speed-loop code don't transfer. The dead-reckoning math and the outer guidance *concept* do.

**Coordinate frame — compass-referenced, not start-relative.** Local Cartesian meters (no lat/lon/UTM anywhere), heading 0° locked to a fixed IMU compass reading — captured exactly once, the first time `odometry.py` reads the IMU (in practice: whichever way the rover is pointed when `mission.py` is *launched*), reused for every mission run within that script's lifetime. **Position** `(0,0)` resets on every SWB MANUAL→AUTO rising edge (each mission starts from wherever the rover currently is), but **heading does not re-lock there** — see scratchpad.md Mistake 11, an actual bug where an earlier version re-locked heading on every restart too, silently defeating the entire point of choosing compass-referenced over start-relative (a pattern would have faced a different real-world direction on every restart). Caught by the user asking a clarifying question, not by the original smoke test.

**Algorithm — Pure Pursuit, not heading-error P/PID.** The BBB's `W`-based steering doesn't map onto Ackermann (no direct angular-velocity actuator, only a steering angle). Pure Pursuit's lookahead-point geometry outputs a steering angle directly (`δ = atan(2·L·sin(α)/Ld)`), which drops straight into `uart_link.py`'s `send_command(steer_target_deg, speed_target_ms)` with no intermediate conversion.

**Tuned parameters, final:**

| Parameter | Value | Basis |
|---|---|---|
| Lookahead `Ld` | 0.8 m | Same as old system's `LOOK_AHEAD_DIST` |
| Waypoint spacing — straight | 1.0 m | Matches old `waypoints_1m.csv` convention |
| Waypoint spacing — curves | 0.3 m | User decision |
| Lawnmower row spacing | 1.0 m | User decision — see finding below |
| Waypoint-reached / mission-end threshold | 0.25 m | Tightened from the old system's 0.4m (RTK-noise-tuned; ours is dead-reckoning-drift-tuned, and our path itself is denser) |
| Heading-error bound (speed shaping) | 25° | Kept as old default, deferred to field tuning |
| Speed-shaping cos floor | 0.2 | Kept as old default (`V *= max(0.2, cos(bounded heading error))`), deferred to field tuning |
| Cruise speed | 0.2 m/s (placeholder) | **Not yet decided by the user** — conservative default, flagged for review |

Since curve spacing (0.3m) is narrower than `Ld` (0.8m) — the opposite relationship the old system had (`Ld` narrower than its 1.0m spacing) — the lookahead point on curves routinely spans several waypoints at once. Implemented proper circle-path intersection search (`guidance.py`) rather than porting the old segment-local projection code, which assumed the lookahead point stays within the current segment.

**⚠️ Physical-feasibility finding: lawnmower turn radius.** A clean single-arc U-turn at 1.0m row spacing needs a 0.5m turn radius. This chassis's actual minimum (wheelbase 0.8m, 45° max lock) is **0.8m** — tighter than the vehicle can steer. `waypoints.py`'s `_semicircle_turn()` catches this (compares against `MIN_TURN_RADIUS_M`) and uses 0.8m instead, landing the turn **1.6m over, not 1.0m**, printing a warning when it does. Confirmed via smoke test: a 4-row/3-turn lawnmower pattern ends up offset by exactly 4.8m (3×1.6m). A true 1.0m in-spacing turn would need a reverse/three-point-turn maneuver — not implemented, would need explicit sizing before building. **Not yet decided by the user** whether this is acceptable or worth building the tighter-turn version.

**File reference:**

| File | Purpose |
|---|---|
| `RPi_companion/rover_config.py` | Shared tunables — mirrors STM32 `ackermann_config.h`/`config.h` geometry constants, plus all the guidance parameters above. No shared header between Python and C; kept in sync by hand |
| `RPi_companion/waypoints.py` | Pattern generators (straight/rectangle/circle/lawnmower) + CSV I/O, pure geometry, no hardware access |
| `RPi_companion/odometry.py` | Dead-reckoning `Pose` estimator — fuses `imu.py` heading + STM32 encoder feedback |
| `RPi_companion/guidance.py` | `PurePursuit` controller — `Pose` + path in, `(steer_target_deg, speed_target_ms)` out |
| `RPi_companion/path_prompts.py` | Shared `input()` helper (float/int prompts with retry-on-bad-input) for the `make_*_path.py` scripts — not a program on its own |
| `RPi_companion/make_straight_path.py`, `make_rectangle_path.py`, `make_circle_path.py`, `make_lawnmower_path.py` | Interactive path generators — prompt for that shape's dimensions, save to `paths/<shape>_<dims>.csv`, print the exact `mission.py --csv ...` command to run it. One CSV-generation concern each, thin wrappers over `waypoints.py` |

**Status:** `mission.py` built, deployed, and **now driven under SWB=AUTO for the first time** — two real straight-line field tests (`mission_20260911_113236`, `_114042`). Every run auto-archives both the path used and a 31-column telemetry log to `~/RPi_companion/logs/` — this logging is what made the following diagnosis possible at all.

### 3.10 First AUTO Field Test — Diverged, Root-Caused, Two Fixes Applied

**Symptom** (user's report): "initially it went straight later turning." Both runs tracked well for the first ~10-12m, then spiraled into a growing steering oscillation — run 1 swung through ~50° of real heading change before happening to re-approach the final waypoint and finish; run 2 diverged further (reached yaw ≈ -156°, essentially backward) before the log ends. Full analysis in scratchpad.md (Mistakes 13/14); summary:

| Cause | Detail | Fix |
|---|---|---|
| **Bug 1 — lookahead search regression** | The "search one segment behind the cursor" logic added to fix the mission-start bug (§3.9) returned the FIRST valid Ld-circle intersection, not the furthest-along one. Near a waypoint-advance boundary, the behind-segment can have its own valid but BACKWARD intersection — hand-verified on a captured glitch: behind-segment gave path-progress 6.02, the correct forward segment gave 7.57. Caused two single-tick spikes (`alpha` briefly hitting -172°/-159°) per run. | `_find_lookahead_point()` now scans every candidate segment and keeps whichever has the greatest cumulative path progress. Regression-tested by replaying the exact captured scenario. |
| **Bug 2 — actuator lag vs. controller's implicit fast-response assumption** | The linear actuator moves at a fixed slow rate (~7mm/s bang-bang, no proportional speed). Pure Pursuit recomputes a full-authority target every 50ms with no model of how fast the actuator can move. Logged: `steer_cmd=-34.0°` while the *actual measured* steering angle was only `-9.7°` at the same tick — a chronic, large lag. Since the rear differential drives off the *actual* (lagging) angle, the rover kept following an old heading while the controller — seeing the resulting position error — demanded ever-larger corrections. **A genuine control-loop instability, not a simple mistuning.** | Not directly fixed — see below. |

**User's own diagnosis was the right call:** "if the error is increasing, the speed should reduce" is a legitimate, well-targeted mitigation for Bug 2 specifically — less distance covered per second gives the slow actuator more real time to catch up per meter of required heading change. While implementing it, found the speed law was **barely responsive at all**: it reused `YAW_BOUND_DEG` (25°, meant only for smoothing the *steering* law) as its own error input, and `cos(25°)=0.906` capped any possible speed reduction at ~9% — explaining why logged speed stayed ~0.18–0.20 m/s almost the entire time, even while `alpha` hit 157°.

**Speed law rebuilt** with independent thresholds:

| Parameter | Value |
|---|---|
| `CRUISE_SPEED_MPS` | 0.20 → **0.12** (felt too fast in the field, per report) |
| `MIN_SPEED_MPS` | 0.05 → **0.06** (small extra margin above the STM32's 0.03 brake deadband) |
| `SPEED_ALPHA_FULL_DEG` | **60°** (new — heading error at which speed bottoms out) |
| `SPEED_CTE_FULL_M` | **0.8** = `LOOKAHEAD_M` (new — cross-track error at which speed bottoms out) |
| `MIN_SPEED_FACTOR` | **0.3** (new — floor fraction of target speed even at max error, so it creeps rather than stops) |

Speed = `target_speed × min(alpha_factor, cte_factor)` — the worse of the two errors wins (not multiplied, which would double-penalize a rover only moderately off on both). **Validated** by replaying the actual bad run's logged trajectory through the new law: speed now drops to the `0.06` floor exactly during the worst of the divergence (`cte=-0.94m`), instead of staying at ~0.18-0.20 throughout as it did on the real run.

**Open, needs a decision:** the speed fix mitigates Bug 2's rate of growth but doesn't address the actuator-lag mismatch itself. A **steering-command slew-rate limiter** on the RPi side (cap how fast `steer_cmd` is allowed to change per tick, matched to what the actuator can plausibly track) would target the actual mechanism directly. Not built yet — additional scope beyond what was asked; worth deciding after seeing whether the speed fix alone is enough in the next field test.

**Not yet re-tested on hardware** — both fixes verified via smoke tests (including exact real-scenario replays) and by replaying the real bad run's trajectory through the new code, but no new AUTO field run has happened since.

**User-confirmed, open items:**
- 1.6m effective lawnmower turn spacing (vs. the nominal 1.0m row spacing) — **accepted**, not a blocker.
- `MAX_STEER_ANGLE_DEG=45°` mirrored into `rover_config.py` is the STM32's *reference/design* value, not yet a verified true mechanical limit — a steering potentiometer is due to be replaced and recalibrated (user's own words: "the angles of turn at front wheel is not 45 degree... the actual angle can be found by using the raw adc values"). **STM32 firmware must not be touched** until that recalibration happens — explicit user instruction. Once recalibrated, update `MIN_TURN_RADIUS_M`'s input (`MAX_STEER_ANGLE_DEG`) in `rover_config.py` to match whatever the real verified value turns out to be.
- Cruise speed (`CRUISE_SPEED_MPS=0.2`) — still an unconfirmed placeholder.

---

## 4. Version Control

**Remote:** `https://github.com/sachu018/Autonomous_Ackermann` (private). Local repo at the project root, initial commit made 2026-09-09 covering everything on disk at that point (STM32 firmware, RPi companion code, ESP32 sniffer, documentation, and the archived BBB-era `Old_files/`).

**Workflow going forward:** after each meaningful code or doc change, a commit is made with a message explaining what changed and why (not just what), then pushed to `origin/main` immediately — no separate "please push" step needed.

**What's excluded** (`.gitignore`): `Rover_closed_loop/Debug/` (STM32CubeIDE build output — regenerated by Build All, not source) and Python `__pycache__/`/`*.pyc`. Everything else, including `Old_files/`'s historical CSV logs, is tracked in full per an explicit decision to keep the complete history rather than trim it.

**Local tooling note:** `gh` (GitHub CLI) isn't part of the base OS image on this dev laptop and there's no passwordless `sudo` here — it was installed as a self-contained binary to `~/.local/bin/gh` (already on `PATH` via `.bashrc`) rather than via a package manager. `gh auth setup-git` was required once, after `gh auth login`, before plain `git push` would work — `gh` storing a token doesn't automatically wire itself into git's credential helper.

---

## 5. Work Breakdown Structure & Phase Status

### Phase 1: Architecture, Review & Planning — [COMPLETED]
- [x] Analyzed `STM_ackermann_backup` modules.
- [x] Verified and documented 7 corrections + 1 design improvement (Kinematic `tanf`, PID derivative-on-measurement, ADC debounce, compile headers, reverse scaling).
- [x] Initialized dedicated tracking in `architecture.md`, `scratchpad.md`, and `works.md`.

### Phase 2: Module Code Integration into `Rover_closed_loop/` — [COMPLETED]
- [x] Added `ackermann_config.h`, `ads1115.h/.c`, `actuator.h/.c`, `steer_pid.h/.c`, `ackermann.h/.c` into `Rover_closed_loop/Core/`.
- [x] Applied verified bug fixes (kinematic `tanf`, PID derivative-on-measurement, ADC debounce hold, reverse scaling, `#include "config.h"`).
- [x] Updated `Rover_closed_loop/Core/Src/main.c` control loop and telemetry.
- [x] Verified `Rover_closed_loop/STM32F411CEUX_FLASH.ld` linker script is clean for GCC 10.

### Phase 3: Building & Firmware Verification — [COMPLETED]
- [x] Clean and build `Rover_closed_loop` project in STM32CubeIDE (0 errors, 0 warnings) — fixed 4 constants dropped during the Phase 2 port (`ACK_STRAIGHT_DEG`, `STEER_INTEGRAL_MAX_PCT`, `KP/KI/KD_STEER`; see scratchpad.md).
- [x] Flashed firmware.

### Phase 4: Step-by-Step Hardware Bring-Up & Calibration
- [x] **Step 1:** ADC feedback verification — done off-board via standalone ESP32 + ADS1115 rig (gain/wiring matched to STM32 firmware); both channels confirmed monotonic and consistent in sign (see scratchpad.md).
- [x] **Step 2:** Potentiometer min/max/center calibration (`ADC_*_MIN/MAX/CENTER_RAW`) — re-applied after both pots were replaced (3rd calibration overall; see scratchpad.md for the full history): L 3096(min)/4426(center)/5793(max), R 2124(min)/3167(center)/4160(max). Spans well-balanced this time (~3-5% split each side) — no repeat of the earlier 3.1x L/R mismatch. **Not yet rebuilt/reflashed** — user to do in STM32CubeIDE. ⚠️ True mechanical lock angle still not verified against `MAX_STEER_ANGLE_DEG` (45°) with a protractor — that constant is unchanged, still the nominal design value, not a measured one.
- [ ] **Step 3:** Actuator polarity & deadband verification — in progress. Actuator confirmed hunting/oscillating at zero on hardware with ±1.3° deadband; widened to ±2.5° (see scratchpad.md). Steering angle calibration replaced twice: first a flat per-wheel zero trim (fixed the zero-offset but broke left pivot, see scratchpad.md Mistake 9), now a proper 3-point (min/center/max) piecewise-linear calibration per wheel — analytically verified (straight=0°/0°, both locks=±45°/±45°). Also found + fixed a persistent left-drift-on-straight bug caused by `ACK_STRAIGHT_DEG` (1.0°) being narrower than `STEER_DEADBAND_DEG` (2.5°), raised to 3.0° (Mistake 10). **None of the above re-tested on hardware after flashing yet.**
- [ ] **Step 4:** Steering PID tuning (`Kp`, `Ki`, `Kd`).
- [ ] **Step 5:** Rear differential test with wheels on blocks.
- [ ] **Step 6:** Ground testing in field conditions.

### Phase 5: Autonomous Integration via Raspberry Pi — [PLANNING]
- [x] Reviewed old BBB autonomous stack (`Old_files/UGV_closed/`) for reusable guidance/localization logic vs. hardware-access code that must be replaced.
- [x] Decided division of responsibility: RPi = guidance/sensing brain, STM32 = real-time actuation server (see §3).
- [x] Decided mode arbitration: SWB repurposed from reverse-speed-cap to MANUAL/AUTO select; UART-link-loss in AUTO mode → brake (see §3.3, §3.5).
- [x] Drafted UART command/feedback frame layout (see §3.4).
- [x] Configured USART2 (PA2/PA3) in `Rover.ioc` / CubeMX — Asynchronous, RX via DMA1_Stream5 + IDLE interrupt (mirrors the USART1/iBUS pattern), code generated and verified against the §10.3 checklist.
- [x] Implemented UART command/feedback framing on the STM32 side (`rpi_link.c/h`) + `AUTO_UART_TIMEOUT_US` failsafe + SWB mode-select wiring in `main.c`. See scratchpad.md for the implementation notes and one regeneration-related regression found and fixed along the way.
- [x] Firmware rebuilt and reflashed to the STM32 with these changes.
- [x] UART link tested end-to-end with a live RPi — see §3.8. 20.0 frames/s, zero checksum failures, correct status bits, confirmed via both `check_stm32_link.py` and `read_encoders.py`, cross-checked with an independent ESP32 sniffer during bring-up.
- [x] IMU hardware-verified — `test_imu.py` shows live, sane heading/roll/pitch/calibration data from the DFRobot Fermion BNO055 on the RPi's I2C1 (see §3.6). Calibration ritual (`calibrate_imu.py`) not yet run.
- [x] Interconnection ported into `Documentations/Rover_study.md` (new §14) as the settled reference version, once verified working — `architecture.md`/`scratchpad.md`/`works.md` remain the working/build-log docs.
- [x] Guidance stack built: `rover_config.py` (shared tunables, mirrors STM32 `ackermann_config.h`), `waypoints.py` (straight/rectangle/circle/lawnmower pattern generation + CSV I/O), `odometry.py` (encoder+IMU dead-reckoning pose), `guidance.py` (Pure Pursuit). All three logic modules verified via standalone smoke tests (no hardware) — not yet deployed/run on the RPi itself (network connectivity to the Pi was down at write time) and not yet wired into a `mission.py` top-level loop that actually calls `uart_link.send_command()`. See §3.9 for the full design discussion and one real physical-feasibility finding (lawnmower turn radius).
- [ ] Wire RTK GNSS to the RPi (replacing the old BBB↔RPi TCP link for RTK). **Not started.**
- [ ] Bench test: RPi sends a *non-neutral* steer/speed target over UART with SWB physically flipped to AUTO on the transmitter, verify the STM32 actually drives the actuator/motors accordingly (only neutral 0°/0 m/s commands and passive listening have been tested so far — the AUTO-mode motion path itself is unexercised).
- [ ] Field test: full autonomous waypoint run.
