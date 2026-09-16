# Rover System — Complete Technical Reference (STM32)

> **Platform:** STM32F411CEU6 "Black Pill" — bare-metal C, STM32Cube HAL
> **Supersedes:** the original BeagleBone Black version of this document. The BBB
> system is retained only as historical/derivation context in §13.
> **Purpose:** Give a new project engineer complete understanding of the rover —
> hardware, firmware, safety, and the failure modes already discovered — so they
> can modify, extend, or debug it independently.
> **Status legend used throughout:** ✅ confirmed working on hardware ·
> ⚠️ known issue / open · 🔍 inferred, not measured
> **§14 covers a different, newer firmware build.** Everything in §1–§13 below
> describes the original **open-loop** `Rover/` project (`pipeline.c` +
> `kinematics.c` differential-drive control). A separate, actively-developed
> project, **`Rover_closed_loop/`**, replaces that control pipeline with
> closed-loop Ackermann front-steering (see `architecture.md` for its full
> design) and adds a Raspberry Pi 5 autonomous companion computer — §14
> documents that RPi↔STM32 interconnection specifically. The two firmware
> builds are siblings, not layers — don't assume anything in §1–§13 about
> `pipeline.c`/`kinematics.c` still applies once `Rover_closed_loop/` is in use.

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Hardware Architecture](#2-hardware-architecture)
3. [STM32F411CEU6 Pin Mapping](#3-stm32f411ceu6-pin-mapping)
4. [Peripheral & Clock Configuration](#4-peripheral--clock-configuration)
5. [Firmware Architecture](#5-firmware-architecture)
6. [Control Pipeline — End to End](#6-control-pipeline--end-to-end)
7. [Startup Sequence](#7-startup-sequence)
8. [Safety Systems](#8-safety-systems)
9. [Known Hardware Details & Unknowns](#9-known-hardware-details--unknowns)
10. [Development Workflow](#10-development-workflow)
11. [Troubleshooting Guide](#11-troubleshooting-guide)
12. [File Reference](#12-file-reference)
13. [Migration Notes: BBB → STM32](#13-migration-notes-bbb--stm32)
14. [Raspberry Pi 5 Interconnection (`Rover_closed_loop/` only)](#14-raspberry-pi-5-interconnection-rover_closed_loop-only)

---

## 1. System Overview

### What is this?

An **agricultural rover** with differential drive — two rear BLDC motors through
20:1 gearboxes, two front castor wheels. Controlled manually via a **FlySky
FS-i6X** RC transmitter. Long-term goal is autonomous waypoint following.

### Why STM32 instead of the BeagleBone Black?

The BBB ran Debian Linux with a Python control stack. **In field conditions the
BBB froze due to heat**, which is unacceptable on a machine that can drive
itself. The STM32F411CEU6 replaces it: no OS, no filesystem, deterministic
timing, far more thermally robust, and much lower power.

### Controller

| | |
|---|---|
| **MCU** | STM32F411CEU6 (Black Pill), Cortex-M4F |
| **Clock** | 100 MHz (HSI 16 MHz → PLL) |
| **Flash / RAM** | 512 KB / 128 KB |
| **Toolchain** | STM32CubeIDE + CubeMX, flashed with STM32CubeProgrammer |
| **Control loop** | 20 Hz, blocking `while(1)` in `main.c` — no RTOS |

### Block Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                      FlySky FS-i6X Transmitter                       │
│      CH1=Steering  CH2=Throttle  CH5=SWC  CH6=SWD  CH7=SWB          │
└─────────────────────────┬───────────────────────────────────────────┘
                          │ 2.4 GHz
                          ▼
┌─────────────────────────────────────────────────────────────────────┐
│                  FlySky FS-iA6B Receiver (iBUS)                      │
└─────────────────────────┬───────────────────────────────────────────┘
                          │ iBUS, 115200 8N1 → PA10 (USART1_RX, DMA)
                          ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    STM32F411CEU6  (bare metal C)                     │
│                                                                      │
│  main.c ──── 20 Hz control loop                                      │
│    ├── ibus.c ────── DMA + IDLE-line iBUS frame parser               │
│    ├── pipeline.c ── steering curve → motion state                   │
│    ├── kinematics.c ─ differential drive (V,W → rpm_L, rpm_R)        │
│    ├── motor.c ───── DAC writes + direction/brake GPIO + contactor   │
│    └── encoder.c ─── TIM2/TIM3 quadrature → wheel RPM                │
│                                                                      │
│  GPIO outputs (all GPIOB, push-pull):                                │
│    PB0  ── Contactor   PB1  ── Rev Left    PB10 ── Brake Left        │
│    PB13 ── Rev Right   PB12 ── Brake Right                           │
│                                                                      │
│  I2C1 @100 kHz (PB6 SCL / PB7 SDA):                                  │
│    0x61 ── DAC Left    0x60 ── DAC Right                             │
│    0x28 ── BNO055 IMU  (planned — not yet implemented)               │
│                                                                      │
│  Encoders (hardware quadrature decode):                              │
│    TIM2 (32-bit) PA0/PA1 ── Left wheel                               │
│    TIM3 (16-bit) PA6/PA7 ── Right wheel   ⚠️ see §9.4                │
│                                                                      │
│  Debug telemetry: PA9 (USART1_TX) ──→ ESP32 ──→ USB serial monitor   │
└─────┬─────────────────────┬─────────────────────────────────────────┘
      │ DAC (I2C)           │ GPIO
      ▼                     ▼
┌──────────────────┐  ┌──────────────────────────────────────────────┐
│ MCP4725 DAC L/R  │  │  Optocoupler / level-shifter board            │
│ 12-bit, 0–5 V    │  │  GPIO HIGH → opto ON → controller input→GND   │
└────────┬─────────┘  └──────────────────┬───────────────────────────┘
         ▼                               ▼
┌─────────────────────────────────────────────────────────────────────┐
│              BLDC Motor Controllers (e-scooter type)                 │
│   Throttle in: 0–5 V analog  ·  Brake in: pull to GND                │
│   Reverse in:  pull to GND   ·  Out: 3-phase to wheel motors         │
└──────────────────────┬──────────────────────────────────────────────┘
                       ▼
┌─────────────────────────────────────────────────────────────────────┐
│  Mechanical: 2× BLDC (400 RPM max) → 1:20 gearbox → wheels           │
│  Wheel radius 0.175 m · Track 0.6 m · Max wheel 20 RPM               │
│  Max linear 0.366 m/s · Max angular 1.22 rad/s                       │
│  Encoders YT06-OP-600B, 600 PPR, mounted on the WHEEL shaft          │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 2. Hardware Architecture

### 2.1 Drive System

| Parameter | Value | Source |
|---|---|---|
| Wheel radius `R` | 0.175 m | `config.h: WHEEL_RADIUS_M` |
| Track width `L` | 0.60 m | `config.h: TRACK_WIDTH_M` |
| Max motor RPM | 400 | `config.h: MOTOR_RPM_MAX` |
| Gearbox | 20:1 | `config.h: GEAR_RATIO` |
| **Max wheel RPM** | **20** | `WHEEL_RPM_MAX` (400/20) |
| Max linear velocity | 0.366 m/s | `config.h: VMAX_MS` |
| Max angular velocity | 1.22 rad/s | `config.h: WMAX_RADS` |
| Spot-turn wheel RPM | 8.75 | `config.h: SPOT_RPM` |

### 2.2 Motor Controller Interface

Each BLDC controller has three inputs:

| Input | Type | Active state | Driven by |
|---|---|---|---|
| **Throttle** | Analog 0–5 V | 0 = stop, 5 V = max | MCP4725 DAC via level shifter |
| **Brake** | Digital | pulled to GND = ON | GPIO → optocoupler |
| **Reverse** | Digital | pulled to GND = ON | GPIO → optocoupler |

**Optocoupler logic:** GPIO **HIGH** → opto conducts → controller input pulled to
GND → function **active**. GPIO LOW → input floats → inactive.

**⚠️ Controller quirk (confirmed on both BBB and STM32):** these controllers will
not engage reverse from an abrupt full-throttle command. They require the
throttle to **ramp up gradually** after a direction change. This is why
`motor.c` ramps the DAC over ~200 ms on every direction change (`_RampDac()`) —
see §9.4. Do not attempt to "fix" this by lengthening `REVERSE_BRAKE_US`; it is
not a timing problem, it is a gradient problem.

### 2.3 DAC (Throttle Generation)

- **MCP4725**, 12-bit, I2C — **Left `0x61`**, **Right `0x60`**, both on **I2C1**
- Output 0–4095 → 0–5 V, through a level shifter to the controller throttle input
- Bus speed **100 kHz (Standard Mode)** — *not* 400 kHz

**Why 100 kHz matters:** the PCB pull-ups are 4.7 kΩ, giving a rise time of
roughly 470 ns. Fast Mode (400 kHz) permits only 300 ns, so the bus produced
framing errors and no ACK. Standard Mode permits 1000 ns. **If CubeMX is ever
regenerated, verify `hi2c1.Init.ClockSpeed == 100000` — it has silently reverted
to 400 kHz before.**

**Write protocol** — MCP4725 "Write DAC Register", **3 bytes**:
```
byte 0: 0x40          command (write DAC register, no EEPROM)
byte 1: D[11:4]
byte 2: D[3:0] << 4
```
Sending only the two data bytes (omitting `0x40`) is interpreted as a *fast
write* command and outputs 0 V forever. This was a real bug — see §11.

### 2.4 FlySky RC System

- **FS-i6X** transmitter → **FS-iA6B** receiver, **iBUS** protocol
- **115200 8N1** into **PA10** (`USART1_RX`), received by **DMA2 Stream2 Ch4**
- 32-byte frames, header `0x20 0x40`, 10 channels ×2 bytes LE, 2-byte checksum
- Checksum = `0xFFFF − sum(first 30 bytes)`
- Frame boundary detected by the **UART IDLE-line interrupt**

| Channel | Index | Control | Function |
|---|---|---|---|
| CH1 | 0 | `Xn` | Steering |
| CH2 | 1 | `Yn` | Throttle |
| CH5 | 4 | SWC | 3-pos forward speed cap |
| CH6 | 5 | SWD | 2-pos arm / disarm |
| CH7 | 6 | SWB | 2-pos reverse speed cap |

Stick range 1000–1500–2000. Deadband ±0.05. RC timeout **400 ms**.

| Switch | Position | Raw | Effect |
|---|---|---|---|
| SWC | Low / Mid / High | <1250 / 1250–1750 / >1750 | 30% / 60% / 100% forward |
| SWB | Low / High | <1500 / >1500 | 60% / 100% reverse |
| SWD | — | >1450 | Arm contactor |

### 2.5 Encoders ✅

- **YT06-OP-600B**, 600 PPR, quadrature ×4 = **2400 counts/rev**
- **Mounted on the WHEEL shaft (after the gearbox)** — so counts are wheel
  revolutions directly, no gear ratio in the maths
- External **4.2 kΩ pull-ups** to 3.3 V on each phase
- Decoded in **hardware** by TIM2/TIM3 in Encoder Mode TI12, input filter = 9

| Wheel | Timer | Width | Pins |
|---|---|---|---|
| Left | **TIM2** | 32-bit | PA0 (CH1) / PA1 (CH2) |
| Right | **TIM3** | **16-bit** ⚠️ | PA6 (CH1) / PA7 (CH2) |

At max wheel speed (20 RPM): 800 counts/s, ~40 counts per 50 ms window →
**resolution 0.5 RPM per count**. Coarse at low speed; adequate for telemetry,
relevant when PID tuning begins.

### 2.6 IMU (BNO055) — planned, not yet implemented

- I2C address **0x28** (0x29 if ADR high) — no conflict with the DACs
- Intended to share **I2C1** (the BBB ran DACs + IMU on one bus successfully)
- Does sensor fusion **on-chip**, so the STM32 only reads finished Euler angles
- Saved calibration offsets from the BBB (`bno055_calibration.json`) can be
  hard-coded — no Flash storage needed:
  `accel [0,0,0]`, `gyro [-2,0,1]`, `mag [111,-187,-126]`,
  `accel_radius 1000`, `mag_radius 461`
- ⚠️ The BNO055 is a known I2C **clock stretcher**. It will share the bus that
  carries throttle commands — see the risk note in §9.4.

### 2.7 Debug / Telemetry Link ✅

```
STM32 PA9 (USART1_TX) ──→ ESP32 DevKit V1 GPIO16 (RX2) ──→ USB ──→ PC
                GND ─────────────── GND
```

The ESP32 runs a pass-through sketch (`esp32_bridge/esp32_bridge.ino`) relaying
UART2 → USB at 115200. Output appears in the Arduino IDE Serial Monitor.

**🚨 NOTHING may be connected to PA10.** PA10 is `USART1_RX` carrying the iBUS
signal. Driving it from the ESP32 collides with the RC link and breaks control
while motors are armed. The bridge is deliberately **one-way**.

---

## 3. STM32F411CEU6 Pin Mapping

**This table is the authoritative pin reference.** It is derived from
`Core/Inc/main.h` and `Rover.ioc`. (An older table in `WorkLog.txt` §2 was stale
and has been corrected — if you find a conflicting table anywhere, trust
`main.h`.)

| Function | Pin | Peripheral / Mode | Notes |
|---|---|---|---|
| **Contactor** | **PB0** | GPIO output PP | HIGH = engaged |
| **Rev Left** | **PB1** | GPIO output PP | HIGH = reverse |
| **Brake Left** | **PB10** | GPIO output PP | HIGH = brake on |
| **Brake Right** | **PB12** | GPIO output PP | HIGH = brake on |
| **Rev Right** | **PB13** | GPIO output PP | HIGH = reverse |
| I2C1 SCL | PB6 | I2C1, AF4, open-drain | DACs (+ future IMU) |
| I2C1 SDA | PB7 | I2C1, AF4, open-drain | 100 kHz |
| iBUS RX | **PA10** | USART1_RX, AF7, DMA | ⚠️ do not connect anything else |
| Debug TX | PA9 | USART1_TX, AF7 | → ESP32 bridge |
| Encoder L A/B | PA0 / PA1 | TIM2_CH1/CH2, AF1 | 32-bit |
| Encoder R A/B | PA6 / PA7 | TIM3_CH1/CH2, AF2 | 16-bit ⚠️ |
| SWD | PA13 / PA14 | SWDIO / SWCLK | debug |
| SWO | PB3 | JTDO-SWO | enabled but **unused** (SWV abandoned) |
| Status LED | PC13 | GPIO output PP | onboard blue LED, active-**LOW** |

**Free pins** (for IMU on a separate bus, GPS, etc.): PA2, PA3, PA4, PA5, PA8,
PA11, PA12, PA15, PB2, PB4, PB5, PB8, PB9, PB14, PB15, PC14, PC15.
*(I2C3 is available on PA8/PB4 if the IMU should be isolated from the DAC bus.)*

---

## 4. Peripheral & Clock Configuration

### 4.1 Clock Tree

```
HSI 16 MHz → PLLM 8  →   2 MHz
           → PLLN 100 → 200 MHz  (VCO)
           → PLLP 2   → 100 MHz  SYSCLK / HCLK
           → PLLQ 4   →  50 MHz  (unused)
APB1 = HCLK/2 = 50 MHz  (timer clock ×2 = 100 MHz)
APB2 = HCLK   = 100 MHz
Flash latency 3
```

**⚠️ Do not casually change this.** Two consequences to know:

1. **TIM5 is the microsecond timebase for the entire firmware.** Prescaler 99
   divides the 100 MHz timer clock to exactly 1 MHz. If HCLK changes, *every*
   delay, timeout, debounce and ramp step silently scales with it. Changing the
   clock **requires** recomputing the TIM5 prescaler in the same step.
2. **USB cannot work at this clock.** USB FS needs exactly 48 MHz from PLLQ; this
   tree gives 50 MHz, and the F411 can only clock USB from PLLQ (no PLLSAI /
   CK48M mux). Valid USB requires a 192 MHz VCO → 96 MHz SYSCLK + 48 MHz USB,
   which would drop HCLK to 96 MHz and require TIM5 prescaler 99 → 95. This is
   why USB CDC telemetry was rejected in favour of the ESP32 bridge.

### 4.2 Timers

| Timer | Width | Role | Config |
|---|---|---|---|
| **TIM2** | 32-bit | Left encoder | Encoder Mode TI12, filter 9, period 0xFFFFFFFF |
| **TIM3** | 16-bit | Right encoder | Encoder Mode TI12, filter 9, period 0xFFFF ⚠️ |
| **TIM5** | 32-bit | µs timebase | Prescaler 99 → 1 MHz, free-running |

Only **TIM2 and TIM5 are 32-bit** on this chip. Both are taken. Any future timer
need inherits a 16-bit counter and its rollover question.

**TIM5 rollover** wraps every 2³² µs ≈ **71.6 minutes**. This is handled
correctly everywhere by always computing *differences* in unsigned arithmetic:

```c
uint32_t elapsed = TIM5->CNT - start;      // ✅ correct across wrap
if (now > deadline) { ... }                // ✗ would break at wrap
```

Keep to that idiom in new code.

### 4.3 I2C1

```c
ClockSpeed      = 100000            // Standard Mode — see §2.3
DutyCycle       = I2C_DUTYCYCLE_2
AddressingMode  = 7-bit
NoStretchMode   = DISABLE           // clock stretching PERMITTED (needed for BNO055)
```

### 4.4 USART1

115200 8N1, `UART_MODE_TX_RX`. **RX** on PA10 via **DMA2 Stream2 Channel 4**
(Normal mode, byte/byte) plus the **IDLE-line interrupt**, NVIC priority 5.
**TX** on PA9 is used for debug telemetry by writing `USART1->DR` directly —
see §5.4 for why HAL is deliberately bypassed there.

---

## 5. Firmware Architecture

### 5.1 Module Map

```
main.c              entry, 20 Hz control loop, debug telemetry,
                    I2C bus recovery, cold-boot handling
├── config.h        ALL tunables and constants (mirrors old BBB config.py)
├── ibus.c/.h       iBUS DMA + IDLE parser → Xn, Yn, switches, rc_ok
├── pipeline.c/.h   Xn/Yn → motion state + wheel RPM + DAC values
│   └── kinematics.c/.h   differential drive (V,W) → (rpm_L, rpm_R)
├── motor.c/.h      DAC writes, direction/brake GPIO, contactor,
│                   I2C recovery, DAC ramp
└── encoder.c/.h    TIM2/TIM3 quadrature → wheel RPM + raw counts
```

CubeMX-managed files: `main.c` (partially), `stm32f4xx_it.c`,
`stm32f4xx_hal_msp.c`. **All hand-written code lives inside `USER CODE` markers**
so regeneration preserves it — but always verify afterwards (§10.3).

### 5.2 The Control Loop (`main.c`)

```
while (1)
  t0 = TIM5->CNT

  1. IBUS_Read()                    → Xn, Yn, caps, swd_on, rc_ok
  2. Contactor edge-detect + 500 ms debounce on SWD
  3. If armed AND rc_ok → Pipeline_Run()
     else               → STATE_DISARMED / STATE_NO_SIGNAL
  4. Drive motors by state:
       BRAKE / NO_SIGNAL → Motor_Brake()
       DISARMED          → Motor_Stop()
       REV (first tick)  → Motor_BrakeThenReverse()
       otherwise         → Motor_SetSpeeds()
  5. Encoder_Update()
  6. Debug telemetry (every 5th tick → ~4 Hz)
  7. Busy-wait until 50 ms has elapsed
```

Single-threaded, no RTOS, no dynamic allocation. The only interrupts are USART1
(iBUS IDLE) and DMA2 Stream2.

### 5.3 Motion States

`STATE_FWD`, `FWD_LEFT`, `FWD_RIGHT`, `REV`, `REV_LEFT`, `REV_RIGHT`,
`SPOT_LEFT`, `SPOT_RIGHT`, `BRAKE`, `DISARMED`, `NO_SIGNAL`

### 5.4 Debug Telemetry ✅

Emitted at ~4 Hz (every 5th loop tick — at 20 Hz a ~70-char line would consume
~10% of every cycle):

```
CMD L+6.00 R+6.00 | ENC L+5.92 R+6.05 | cnt 412 419
```

| Field | Meaning |
|---|---|
| `CMD` | commanded wheel RPM from the pipeline |
| `ENC` | measured wheel RPM from the encoders |
| `cnt` | raw quadrature counter values (L, R) |

Printing commanded and measured together answers wiring, direction and maths
correctness in one line — and is exactly the signal needed for PID tuning later.

**Two deliberate implementation choices, both important:**

**`_DbgPutc()` writes `USART1->DR` directly, never `HAL_UART_Transmit()`.**
HAL takes `__HAL_LOCK(huart1)` and holds it for the whole blocking send; if the
iBUS IDLE interrupt fires during that window, `HAL_UART_Receive_DMA()` inside
`IBUS_IdleCallback()` returns `HAL_BUSY`, the RX DMA never restarts, and RC
frames drop. With frames every ~7 ms this would collide constantly. TX and RX are
independent paths in the USART, so a direct register write is safe and never
touches the lock. The `TXE` wait is hardware-bounded (~87 µs/char) so it cannot
stall the loop, even with nothing connected to PA9.

**`_FmtRpm()` avoids `%f`.** newlib-nano omits float formatting unless
`-u _printf_float` is added to the linker flags, and silently prints nothing or
garbage otherwise. Values are formatted via scaled integers instead.

---

## 6. Control Pipeline — End to End

Worked example: stick right 0.30, half throttle forward 0.50, SWC mid (60%).

```
[1] ibus.c — IBUS_IdleCallback() on IDLE line
    frame validated (header + checksum) → channels
    Xn = _ToFloat(ch[0]) = +0.30
    Yn = _ToFloat(ch[1]) = +0.50
    fwd_cap = 0.60   rev_cap = 1.00   swd_on = 1   rc_ok = 1
        │
        ▼
[2] main.c — contactor already armed (SWD rising edge + 500 ms debounce)
        │
        ▼
[3] pipeline.c — Pipeline_Run(0.30, 0.50, 0.60, 1.00, 1)
    not BRAKE (sticks not neutral), not SPOT (Yn ≠ 0)
    going_fwd = 1, safe_pct = 0.60

    V       = 0.50 × 0.366              = 0.183 m/s
    w_scale = (0.30³)/(0.50 + 0.6) × 0.6 = 0.0147
    W       = −0.0147 × 1.22             = −0.0180 rad/s   (Xn>0 → right)
    _ApplySafety: |V| < 0.366×0.60 → unchanged; |V|>0.1 → W ×= 0.60
        │
        ▼
[4] kinematics.c — Kinematics_ComputeRPM(V, W)
    clamp W to ±2|V|/L so the inner wheel never reverses in an arc
    VL = V − (L/2)·W        VR = V + (L/2)·W
    rpm = (Vwheel / (2πR)) × 60
    scale both down if either exceeds WHEEL_RPM_MAX (20)
    enforce MIN_RATIO (0.2) on the inner wheel to prevent stall
        │
        ▼
[5] pipeline.c — _ToThrottle(rpm) = |rpm| / 20.0 × 4095   → dac_L, dac_R
    state = STATE_FWD_RIGHT
        │
        ▼
[6] motor.c — Motor_SetSpeeds(rpm_L, rpm_R, dac_L, dac_R)
    direction unchanged → set REV GPIOs, release BRAKE GPIOs
    wait MOTOR_SEQ_DELAY_US (20 ms)
    _WriteDac(0x61, dac_L)    _WriteDac(0x60, dac_R)

    (on a DIRECTION CHANGE instead: zero DACs → 10 ms → toggle REV GPIOs →
     150 ms settle → 20 ms → _RampDac() ramps 0→target over ~200 ms)
        │
        ▼
[7] DAC outputs → level shifter → controller throttle inputs → wheels turn
        │
        ▼
[8] encoder.c — Encoder_Update() reads TIM2/TIM3, computes measured RPM
        │
        ▼
[9] main.c — debug line out on PA9 → ESP32 → serial monitor
```

---

## 7. Startup Sequence

```
Power applied
  │
  ├─ HAL_Init()
  │
  ├─ [USER CODE Init/SysInit]
  │   ├─ If RCC_FLAG_PORRST or BORRST (genuine COLD boot only):
  │   │     HAL_Delay(3000)          ← let the DAC supply rail settle
  │   │   (a later manual RESET press is RCC_FLAG_PINRST → skipped)
  │   ├─ __HAL_RCC_CLEAR_RESET_FLAGS()
  │   └─ _I2C1_BusRecovery()          ← PB6/PB7 as raw GPIO:
  │         release SDA, clock SCL ×9, manual STOP
  │         (must run BEFORE MX_I2C1_Init() claims those pins)
  │
  ├─ SystemClock_Config()             → 100 MHz
  │
  ├─ MX_GPIO_Init()   ← all motor GPIOs LOW (safe state)
  ├─ MX_DMA_Init()    ← MUST precede peripherals that use DMA
  ├─ MX_I2C1_Init()   ← 100 kHz
  ├─ MX_TIM2_Init()  MX_TIM3_Init()   ← encoder mode
  ├─ MX_USART1_UART_Init()
  ├─ MX_TIM5_Init()   ← µs timebase
  │
  ├─ Motor_Init()
  │   ├─ _DiagLedInit()        PC13 off
  │   ├─ _WaitForDacsReady()   poll HAL_I2C_IsDeviceReady, 300 ms cap
  │   ├─ all control GPIOs LOW
  │   └─ zero both DACs
  ├─ Encoder_Init()   start TIM2/TIM3 encoders + TIM5 base
  ├─ IBUS_Init()      enable IDLE IT, start RX DMA
  │
  ├─ Motor_Energize()          ← AUTO-ARM on boot
  │   ├─ _EnsureDacsZeroed()   best-effort (never blocks arming)
  │   ├─ Contactor GPIO HIGH   ← audible click = "system ready"
  │   ├─ 500 ms settle
  │   └─ _EnsureDacsZeroed()   ← THE important one: first moment the DAC
  │                              can respond if it is powered downstream
  │                              of the contactor. Verifies + retries with
  │                              bus recovery. Clears PC13 on success.
  │
  └─ Enter 20 Hz control loop
```

**Boot behaviour to expect:** on a cold power-up there is a ~3 s pause, then the
contactor clicks. On a manual reset the click is immediate. **PC13 lit after the
contactor closes = the DACs could not be reached** (I2C/DAC hardware fault).

**⚠️ Note the rover AUTO-ARMS on boot**, unlike the BBB which started disarmed.
The contactor click is the deliberate "firmware alive" indicator.

---

## 8. Safety Systems

### 8.1 Physical

| Feature | Mechanism |
|---|---|
| Main contactor | Relay on PB0 cuts power to the motor controllers |
| Contactor arming | SWD edge-detect + 500 ms debounce (`CONTACTOR_DEBOUNCE_US`) |
| Safe GPIO state | `MX_GPIO_Init()` + `Motor_Init()` drive everything LOW first |

### 8.2 Software

| Feature | Mechanism | Location |
|---|---|---|
| RC signal loss | 400 ms without a valid frame → `rc_ok = 0` → BRAKE | `ibus.c: IBUS_Read` |
| Sticks neutral | `Xn == 0 && Yn == 0` → BRAKE | `pipeline.c` |
| No RC → no motion | `Motor_IsArmed() && rc.rc_ok` gates the pipeline | `main.c` |
| Speed caps | SWC / SWB scale V and W | `pipeline.c: _ApplySafety` |
| Disarm | SWD off → `Motor_Stop()` + `Motor_Deenergize()` | `main.c` |
| Reverse brake | 500 ms brake before engaging reverse | `motor.c` |
| Inner-wheel clamp | W limited so the inner wheel never reverses in an arc | `kinematics.c` |

### 8.3 ⚠️ Known Safety Gaps (open)

**1. Blocking delays stall the failsafe.**
A reverse manoeuvre with a direction change blocks the loop for roughly:
`500 ms brake + 10 + 150 + 20 + ~200 ms ramp ≈ 880 ms`. During that window
`IBUS_Read()` is never called, so **the 400 ms RC-loss failsafe cannot
evaluate**. A `_DelayUsSafe()` that polls the RC link and aborts to brake was
written and then reverted during a rollback; it was never independently tested.

**2. DAC write failures are ignored while driving.**
`Motor_SetSpeeds()` uses the fire-and-forget `_WriteDac()`, which discards the
HAL status. If the I2C bus wedges mid-drive, **the DAC holds its last value and
the wheels keep turning**. The checked variant (`_WriteDacChecked`) already
exists and is used in `_EnsureDacsZeroed()` — the driving path simply does not
use it. This becomes more pressing once the BNO055 (a known clock-stretcher)
joins the same bus.

**3. Halting the debugger is not a stop button.**
If you halt the core with motors armed, the DAC keeps its last value and the
wheels keep turning while the CPU is stopped. **Never set breakpoints while
armed.** SWD-off on the transmitter is the real stop.

---

## 9. Known Hardware Details & Unknowns

### 9.1 Verified ✅

| Item | Detail |
|---|---|
| MCU | STM32F411CEU6, 100 MHz, HSI+PLL |
| DAC | MCP4725 ×2, L = 0x61, R = 0x60, I2C1 @100 kHz |
| DAC protocol | 3-byte write, command `0x40` |
| Encoders | YT06-OP-600B, 600 PPR ×4 = 2400 cnt/rev, **wheel shaft** |
| Encoder decode | TIM2 (32-bit) left, TIM3 (16-bit) right, TI12, filter 9 |
| RC | FlySky iBUS, 115200, PA10, DMA + IDLE interrupt |
| Geometry | R 0.175 m, L 0.60 m, gear 20:1, wheel max 20 RPM |
| Telemetry | PA9 → ESP32 → USB serial @115200 |
| Drive | forward, reverse, left, right, spot turns all functional |

### 9.2 Inferred, not measured 🔍

| Item | Reasoning |
|---|---|
| **DAC powered downstream of the contactor** | Best explanation for the cold-boot failure and the deadlock it caused. **Never confirmed with a multimeter** — see §9.5 |
| Contactor | Mechanical relay, normally open |
| Optocouplers | Standard GPIO→controller isolation (e.g. PC817) |
| Level shifter | Buffers DAC 0–5 V to the controller throttle input |

### 9.3 Still Unknown

| Item |
|---|
| BLDC controller brand / model / exact specs |
| Optocoupler and level-shifter schematics |
| Contactor coil voltage and drive circuit |
| Battery / regulator rail voltages |
| Motor controller minimum throttle threshold |
| Hardware E-stop — exists or software only? |
| Fuse ratings, cable gauge |

### 9.4 ⚠️ Open Technical Issues

**TIM3 16-bit encoder overflow (right wheel) — FIXED, awaiting hardware test.**
TIM3 is 16-bit, so its counter wraps every **~82 s** of continuous rotation at
full wheel speed (~4.5 min at the 30% cap). Naive subtraction gives
`delta = 0 − 65535` at the wrap, which the RPM maths turns into exactly one
bogus **±32768 RPM** sample:

```
65536 × 1200 / 2400 = 65536 / 2 = 32768
```

The left wheel (TIM2, 32-bit) would not wrap for ~2 months.

*Fix implemented:* handled **arithmetically** in `_Delta16()` (`encoder.c`) —
take the counter difference modulo 2¹⁶ and read the upper half of the range as
negative. No interrupt, no overflow counter, no flag handling, no critical
section. Correct as long as the wheel moves <32767 counts between samples;
actual is ~40, so roughly 800x margin.

*⚠️ Do NOT reintroduce a TIM3 interrupt.* `HAL_TIM_Encoder_Start_IT()` enables
the CC1/CC2 *edge* interrupts, not Update — an ISR that only clears the Update
flag never clears the flag that woke it and re-fires forever. That locked the
MCU solid. A warning comment sits in `stm32f4xx_it.c` where the handler used
to be.

**BNO055 on the throttle bus.** The IMU is a known clock-stretcher and would
share the bus that carries throttle commands. If it hangs holding SCL low it
wedges the DAC path. I2C3 (PA8/PB4) is free if isolation is preferred later.

**Unproven boot-time code.** The 3 s cold-boot delay, `_I2C1_BusRecovery()` and
`_WaitForDacsReady()` are all present and the system works — but their
*individual* contributions were never isolated. Any may be dead weight. If
simplifying, **remove one at a time with a cold-boot test between each.**

### 9.5 Recommended Measurement

**Measure MCP4725 Vdd with the contactor OPEN vs CLOSED.** If it reads 0 V with
the contactor open, that confirms the inference in §9.2, closes a long-standing
unknown, and would justify simplifying the boot path (all DAC setup would
belong strictly after energising).

---

## 10. Development Workflow

### 10.1 Build

STM32CubeIDE → **Project → Build All**. The project imports via
**File → Open Projects from File System…** pointed at `stm/Rover`
(it contains `.project` / `.cproject`).

### 10.2 Flash

**Use STM32CubeProgrammer** (SWD, connect **Under reset**). The CubeIDE debug
launcher has been unreliable on this setup — `Failed to start GDB server /
Failed to connect to device` — even though the same ST-Link flashes fine from
CubeProgrammer.

**Only one application may claim the ST-Link at a time.** If CubeProgrammer is
connected, CubeIDE cannot attach, and vice versa. Disconnect and close one
before using the other.

### 10.3 ⚠️ After Any CubeMX Regeneration — Verify

Regeneration has silently broken this project before. Always check:

| Check | Expected |
|---|---|
| `hi2c1.Init.ClockSpeed` | `100000` — **not** 400000 |
| `stm32f4xx_it.c` | exactly **one** `USART1_IRQHandler` |
| USER CODE blocks | `IBUS_IdleCallback`, cold-boot delay, `_I2C1_BusRecovery()` all still present |
| `HAL_UART_MspInit` | USART1 NVIC enable still present |
| Code placement | CubeMX may relocate code between `USER CODE Init` and `SysInit` — harmless here, but verify ordering is still before `MX_I2C1_Init()` |

### 10.4 Reading Telemetry

1. Upload `esp32_bridge/esp32_bridge.ino` to the ESP32 (Board: **ESP32 Dev
   Module**)
2. Wire **PA9 → GPIO16** and **GND → GND** *(nothing to PA10)*
3. Serial Monitor at **115200**

*WROVER modules use GPIO16/17 for PSRAM — use GPIO25/26 there instead.*

### 10.5 Working Practice (learned the hard way)

**One change per build-flash-test cycle.** Batching several speculative changes
into one flash caused the two worst regressions in this project's history: when
the result got worse, there was no way to tell which change was responsible, and
one of them had to be rolled back wholesale. Change one thing, test, then move
on.

---

## 11. Troubleshooting Guide

### Symptom → Cause

| Symptom | Likely cause | Fix |
|---|---|---|
| **No contactor click at boot** | Firmware not running / not flashed | Re-flash; check power |
| **Contactor clicks, PC13 lit, no movement** | DACs unreachable over I2C | Check DAC power, PB6/PB7 wiring, address strapping |
| **Contactor + RC work, no movement at all** | I2C bus wedged (historic cold-boot bug) | Should be fixed; if it returns, see §9.5 |
| **Reverse does nothing, forward fine** | *(historic)* unsigned wraparound in `_ToFloat()` | Fixed — cast to `int32_t` before subtracting |
| **Left steering does nothing, right fine** | Same bug as above | Fixed |
| **Reverse needs a slow stick pull** | Controller rejects abrupt reverse throttle | Fixed by `_RampDac()` — do not tune `REVERSE_BRAKE_US` |
| **MCU freezes when a wheel turns** | *(historic)* TIM3 interrupt storm | Never re-enable `TIM3_IRQn` without servicing CC1/CC2 |
| **One `ENC R ±32768` spike periodically** | TIM3 16-bit wrap — **expected**, not a fault | See §9.4 |
| **`ENC` sign opposite to `CMD`** | Encoder A/B phases swapped | Swap the two wires, or flip in software |
| **One `ENC` stays 0.00, `cnt` frozen** | That encoder not wired / unpowered | Check wiring and common GND |
| **`cnt` moves but `ENC` reads 0.00** | Too slow — `ENC_STOPPED_US` (500 ms) zeroes it | Increase speed |
| **RC unresponsive** | DMA/NVIC/IDLE config | Verify DMA2 Stream2 + USART1 NVIC in MSP |
| **Nothing in Serial Monitor** | Bridge wiring or baud | Bridge prints a diagnostic after 3 s of silence |
| **CubeIDE debug won't launch** | ST-Link claimed by CubeProgrammer | Close CubeProgrammer; use **Under reset** |

### Historical Bug Index

Full write-ups are in `WorkLog.txt`. Summary of what has already been fixed —
useful because several were subtle and could be reintroduced:

| # | Bug | Root cause |
|---|---|---|
| 1 | No contactor click at boot | No auto-energise; added `Motor_Energize()` at boot |
| 2 | iBUS not receiving | Missing USART1 NVIC; DMA not configured; duplicate IRQ handler |
| 3 | Throttle stuck at 0 V | Missing MCP4725 `0x40` command byte; I2C at 400 kHz with 4.7 kΩ pull-ups |
| 4 | Reverse + left dead | `raw - IBUS_VAL_MID` computed **unsigned** → `1000u−1500u` wrapped to ~4.29e9 |
| 5 | Total MCU freeze | TIM3 ISR cleared only the Update flag while CC1/CC2 kept re-firing |
| — | Cold boot no movement | Early I2C writes to a not-yet-ready DAC wedged the bus; fixed by verifying + retrying DAC zeroing **after** the contactor closes |
| — | Total lockout (self-inflicted) | Gating the contactor on I2C success deadlocked: DAC needs power, power needs contactor, contactor waited on DAC |

**Two lessons worth carrying forward:**
- **Never discard a HAL status you could act on.** `_WriteDac()` throwing away
  `HAL_I2C_Master_Transmit()`'s return value is what made the cold-boot bug take
  so long to find — the firmware could not tell its own writes were failing.
- **Never gate arming on an I2C transaction.** Interlocks must fail in a
  direction the operator can recover from.

---

## 12. File Reference

### STM32 Firmware — `stm/Rover/`

| File | Purpose | Status |
|---|---|---|
| `Core/Inc/config.h` | All tunables, pin-independent constants | ✅ |
| `Core/Src/main.c` | Entry, 20 Hz loop, telemetry, I2C recovery, cold-boot | ✅ |
| `Core/Inc/main.h` | **Authoritative GPIO pin definitions** | ✅ |
| `Core/Src/ibus.c` `.h` | iBUS DMA + IDLE parser | ✅ |
| `Core/Src/pipeline.c` `.h` | Steering curve → motion state | ✅ |
| `Core/Src/kinematics.c` `.h` | Differential drive maths | ✅ |
| `Core/Src/motor.c` `.h` | DAC, GPIO, contactor, I2C recovery, ramp | ✅ |
| `Core/Src/encoder.c` `.h` | Quadrature → RPM + raw counts | ✅ ⚠️ TIM3 overflow |
| `Core/Src/stm32f4xx_it.c` | IRQ handlers (IDLE hook in USER CODE) | ✅ |
| `Core/Src/stm32f4xx_hal_msp.c` | Peripheral MSP init, NVIC | ✅ |
| `Rover.ioc` | CubeMX project — **pin source of truth with `main.h`** | ✅ |

### Support

| File | Purpose |
|---|---|
| `esp32_bridge/esp32_bridge.ino` | ESP32 UART→USB telemetry bridge (Arduino IDE) |
| `documentations/WorkLog.txt` | Chronological engineering log, full bug write-ups |
| `documentations/Rover_study.md` | **This document** |
| `stm/BBB/` | BeagleBone Black backup — reference only, not deployed |

### Not Yet Implemented

| Module | Purpose |
|---|---|
| `imu.c` / `imu.h` | BNO055 over I2C — Euler angles, calibration restore |
| `pid.c` / `pid.h` | Closed-loop speed control (BBB used Kp=50, Ki=15) |
| GPS | u-blox ZED-F9P over a second UART |
| Wireless telemetry | ESP32 WiFi instead of tethered USB |

---

## 13. Migration Notes: BBB → STM32

### What Each BBB Component Became

| BBB (Python / Linux) | STM32 (bare-metal C) |
|---|---|
| `main.py` 20 Hz loop + `time.sleep()` | `main.c` `while(1)` + TIM5 busy-wait |
| `config.py` | `config.h` (`#define`s) |
| `pipeline.py` | `pipeline.c` |
| `utils/kinematics.py` | `kinematics.c` |
| `drivers/flysky_receiver.py` (pyserial) | `ibus.c` (USART1 + DMA + IDLE IRQ) |
| `drivers/motor_controller.py` (smbus2 + Adafruit_BBIO) | `motor.c` (I2C HAL + GPIO HAL) |
| eQEP via sysfs counters | TIM2/TIM3 hardware encoder mode |
| systemd `robot.service` → `startup.sh` | firmware runs from reset — no init system |
| UDP telemetry → `localhost:5005` | UART → ESP32 → USB serial |
| `bno055_calibration.json` on disk | constants compiled into firmware |
| `config-pin` pinmux at boot | CubeMX-generated `MX_GPIO_Init()` |

### Behavioural Differences

| | BBB | STM32 |
|---|---|---|
| Arming at boot | starts **disarmed** | **auto-arms** (contactor click = ready) |
| Cold-boot delay | none | ~3 s on genuine power-on |
| Telemetry | UDP + web dashboard | 4 Hz serial line |
| IMU | working | **not yet ported** |
| Closed-loop PID | written, unused | **not yet ported** |

### Things That Only Broke in C

Bugs that did not exist in the Python original and were introduced purely by the
port — worth knowing when porting the remaining modules:

- **Signed/unsigned arithmetic.** Python integers are arbitrary-precision and
  signed; `raw - IBUS_MID` just works. In C, `1000u - 1500u` wraps to ~4.29
  billion. **The BNO055 returns signed 16-bit values — assemble them as
  `int16_t`, not `uint16_t`, or this bug returns.**
- **Fixed-width counter rollover.** Linux eQEP counters were 32-bit via sysfs;
  TIM3 is 16-bit and wraps every ~82 s.
- **Interrupt flag management.** No equivalent concept in the Python driver
  stack; getting it wrong froze the MCU entirely.
- **Silent error returns.** `smbus2` raises exceptions on I2C failure; the HAL
  returns a status code that is easy to ignore — and was ignored, at length.

---

## 14. Raspberry Pi 5 Interconnection (`Rover_closed_loop/` only)

**This section describes `Rover_closed_loop/`, not the `Rover/` project the rest
of this document covers.** See the note at the top of this document. Full
design rationale and the day-by-day build/debug log live in `architecture.md`,
`scratchpad.md`, `works.md` at the project root — this section is the settled
reference version of that work, condensed to this document's style.

### 14.1 Why

Autonomous waypoint following (the long-term goal stated in §1) needs
localization, sensor fusion, and a guidance law — real computational work a
bare-metal 100 MHz Cortex-M4 isn't the place to do alongside a 20 Hz real-time
control loop. A Raspberry Pi 5 is added as a companion computer to own that
layer, while the STM32 keeps owning everything safety-critical and real-time
that it already does (steering PID/bang-bang control, the Ackermann electronic
differential, contactor arming, RC failsafes) — the RPi becomes a second
*command source*, not a replacement for the STM32's control authority.

### 14.2 Physical Link ✅

STM32 **USART2** ↔ Raspberry Pi 5 primary UART, GND common:

| STM32 | RPi 5 |
|---|---|
| PA3 (USART2_RX) | Pin 8 / GPIO14 (TXD) |
| PA2 (USART2_TX) | Pin 10 / GPIO15 (RXD) |
| GND | GND (any GND pin) |

115200 8N1, DMA + IDLE-line framing on the STM32 side (`rpi_link.c`, mirrors
`ibus.c`'s pattern for the RC link on USART1) — see §14.7 for the frame
formats.

**⚠️ RPi 5 OS-level prerequisites — not obvious, cost real debugging time to
find:**
1. `GPIO14`/`GPIO15` carry the Linux login console by default on Raspberry Pi
   OS. Must be freed: disable the `console=serial0,...` kernel parameter and
   the `serial-getty` unit for whichever `ttyAMA` node `/dev/serial0` resolves
   to (varies by Pi/image — was `ttyAMA10` on the unit this was built against,
   not the `ttyAMA0` older Pi docs assume).
2. **The Pi 5's onboard Bluetooth shares this exact same UART.** Even after
   step 1, Bluetooth's own boot-time attach process (`hci_uart_bcm`,
   independent of the console) re-claims the port. Fix: `dtoverlay=disable-bt`
   in `/boot/firmware/config.txt` — this disables Bluetooth on the Pi
   permanently, an accepted trade-off since BT isn't used anywhere here.
3. **`enable_uart=1` in `config.txt` — this was the actual root cause of an
   extended "zero bytes arriving" debugging session, not step 1 or 2.**
   Without it, `GPIO14`/`GPIO15` are never switched into UART
   alternate-function mode at the hardware level, even though `/dev/ttyAMA10`
   exists as a kernel device node and a getty may even have run on it
   previously. **Neither a device node existing nor a systemd unit reporting
   "active" proves the GPIO pins are actually hardware-muxed to the
   peripheral** — verify directly with `pinctrl get 14,15` (expect
   `GPIO14 = TXD0`, `GPIO15 = RXD0`; `none`/`none` means step 3 is missing).

### 14.3 Division of Responsibility

| Layer | Runs on | Owns |
|---|---|---|
| Guidance & sensing | Raspberry Pi 5 | IMU + GNSS fusion, localization, path/waypoint management, guidance law → target steering angle + speed |
| Real-time actuation | STM32F411 (`Rover_closed_loop/`) | Steering PID/bang-bang, ADS1115 feedback, Ackermann differential, encoder feedback, contactor/arming, all RC failsafes |

### 14.4 Mode Arbitration — SWD / SWB

Transmitter switches, read via the existing iBUS link on **PA10** (same
physical RC receiver and pin as §2.4/§3 — unchanged):

- **SWD** — arm/disarm the contactor. Same role as the base `Rover/` project.
- **SWB** — **repurposed** in `Rover_closed_loop/` from its `Rover/` role
  (§2.4: reverse-speed-cap). Here it's a pure **MANUAL / AUTO** command-source
  select: below `SWB_THRESH` → MANUAL (RC sticks, as normal); at/above it →
  AUTO (RPi command, only while the link is also healthy — §14.6). Switch-down
  is MANUAL by default, the safer resting state if SWB is never touched.
  Reverse-speed-cap is gone as a consequence — reverse now runs at the same
  cap forward does.

### 14.5 UART Protocol ✅ (verified working end-to-end on hardware)

20 Hz, binary, fixed-size frames, little-endian, matching the STM32's own
control-loop rate.

**Command frame, RPi → STM32, 8 bytes:**

| Byte(s) | Field | Meaning |
|---|---|---|
| 0–1 | header | `0xAA 0x55` |
| 2–3 | `int16` steer_target | degrees × 100, −4500..+4500 |
| 4–5 | `int16` speed_target | mm/s, signed |
| 6 | `uint8` seq | rolling counter (parsed, not yet consumed by the STM32) |
| 7 | `uint8` checksum | XOR of bytes 0–6 |

**Feedback frame, STM32 → RPi, 12 bytes — sent every tick regardless of
MANUAL/AUTO or armed state:**

| Byte(s) | Field | Meaning |
|---|---|---|
| 0–1 | header | `0xBB 0x66` |
| 2–3 | `int16` angle_L | degrees × 100 |
| 4–5 | `int16` angle_R | degrees × 100 |
| 6–7 | `int16` rpm_L | RPM × 10 |
| 8–9 | `int16` rpm_R | RPM × 10 |
| 10 | `uint8` status | bitfield: bit0 ARMED, bit1 STEER_FAULT, bit2 AUTO_ACTIVE, bit3 RC_OK |
| 11 | `uint8` checksum | XOR of bytes 0–10 |

### 14.6 Failsafe ✅

`AUTO_UART_TIMEOUT_US` (300 ms) — same failsafe class as the existing 400 ms
RC-loss timeout (§8.2). If SWB is AUTO and no valid command frame has arrived
within that window, the STM32 forces a brake regardless of the last command
received, independent of what the RPi is doing. SWB back to MANUAL always
hands control back to the RC sticks immediately, independent of UART link
state.

### 14.7 Guidance & Motion Stack (RPi-side) ✅ field-tested

The guidance/localization loop that §14.8 of the earlier revision of this
document flagged as "not yet built" now exists and has been through several
rounds of real field testing. No RTK/GNSS — position is dead-reckoned from
STM32 encoder feedback + IMU heading only, the same class of approach the
BBB-era scripts (`Old_files/dev_bak/`) used.

- **`odometry.py`** — fuses encoder distance (`(rpm_L+rpm_R)/2`, via
  `uart_link.py`'s feedback frame) with IMU heading (`imu.py`, DFRobot Fermion
  BNO055) into a running `(x, y, yaw)` pose, compass-referenced. The heading
  "zero" reference locks **once**, from the **circular mean of
  `HEADING_LOCK_SAMPLES` (30, ~1.5s @ 20Hz) consecutive IMU readings**, the
  first time `update()` is called in a `mission.py` process — not once per
  mission restart (`reset()` only re-zeros position, deliberately). The
  averaging (rather than a single sample) is a mitigation, not a full fix,
  for an intermittent heading-lock bias traced to the BNO055's magnetometer
  never calibrating on this chassis (`imu_calib_mag`/`imu_calib_sys` read 0
  in every field log collected) — see §14.11.
- **`guidance.py`** — Pure Pursuit path follower. Chosen over the BBB's
  heading-error-PID approach specifically because its output *is* a steering
  angle, mapping directly onto `uart_link.py`'s
  `send_command(steer_target_deg, speed_target_ms)` with no intermediate
  V,W→wheel-RPM conversion. Speed is shaped down (toward `MIN_SPEED_FACTOR`
  of the target) as heading error (`alpha`) or cross-track error (`cte`)
  grows, using its own thresholds (`SPEED_ALPHA_FULL_DEG`,
  `SPEED_CTE_FULL_M`) independent of the steering law's own error bound.
- **`waypoints.py`** — path generators (straight/rectangle/circle/lawnmower)
  + CSV I/O. Clamps lawnmower row-turn radius to `MIN_TURN_RADIUS_M`
  (`= WHEELBASE_M / tan(MAX_STEER_ANGLE_DEG)`) rather than silently emitting
  an undrivable path.
- **`mission.py`** — the runnable entry point. Always computes and sends
  commands at 20Hz regardless of SWB position (matches `rpi_link.c`'s
  "STM32 ignores AUTO commands unless SWB is AUTO" safety design — inert
  while MANUAL, not merely idle) and treats SWB's MANUAL→AUTO transition as
  the mission-(re)start trigger, not script-launch time. Auto-archives each
  run's path to `logs/mission_<timestamp>_path.csv` and logs a 31-column
  telemetry CSV (`logs/mission_<timestamp>.csv`) — pose, guidance internals
  (`cte_m`, `alpha_deg`, `lookahead_x/y_m`, `target_idx`), full raw IMU
  (heading/roll/pitch/gyro/accel/all 4 calibration fields), and STM32
  feedback (`angle_L/R`, `rpm_L/R`, status bits) every tick — this is what
  every field diagnosis in §14.11 was worked out from.
- **`rover_config.py`** — shared RPi-side tunables, hand-kept in sync with
  `ackermann_config.h`/`config.h` (no shared header between the two
  languages/processors). Current speed profile: `CRUISE_SPEED_MPS = 0.24`,
  `MIN_SPEED_MPS = 0.12` (doubled from an initial 0.12/0.06 once the
  closed-loop wheel PI — §14.8 — was field-verified working).
- **`calibrate_steering_pots.py`** — field recalibration tool for the
  steering potentiometers (§14.11). Live `angle_L`/`angle_R`/`delta`
  readout over the existing UART link, walks full-left-lock/full-right-lock/
  center; recovers true raw ADC at each captured position by exactly
  inverting `_MapToAngle()`'s own formula with the *current* firmware
  constants (no raw-ADC field exists in the UART frame, so this avoids
  needing one), rather than requiring a new debug channel.

### 14.8 Closed-Loop Wheel-Speed & Steering-Actuator Control (STM32-side) ✅

Two of §8/§9's original open-loop assumptions were replaced this phase, both
after field data showed the open-loop version didn't hold up:

**Rear-wheel throttle — closed-loop PI, forward only.** The original
`_ToThrottle()` linear RPM→DAC map (§2.3-style open-loop guess) was field-
measured to be wrong on both its floor and its slope — actual wheel speed
ran 2.8-5.5× the commanded speed depending on the run, and a first attempt
to fix the floor alone made it *worse* (raising a line's intercept while
holding the far endpoint fixed lifts the whole line, not just the low end).
Replaced entirely for **forward** driving with `wheel_pid.c` — a PI loop
(`Kp=50, Ki=15`, carried over directly from the BBB-era
`Old_files/dev_bak/ugv_pid_waypoint.py`'s `BBBHardware`, same motors/gearbox/
DAC hardware) closing on live `Encoder_GetRPM_L()/R()` feedback each tick,
sidestepping the need for an accurate static curve at all — including under
terrain/traction variation, which a static curve fundamentally can't track.
`THR_FWD_MIN_DAC` is now just a non-critical starting floor the integral
term corrects away from. **Reverse driving is unchanged**, still the
original open-loop `_ToThrottle()` with `THR_REV_MIN_DAC`/`THR_REV_MAX_DAC`
— a deliberate scope decision, not an oversight. `WHEEL_RPM_MAX` (the
forward RPM ceiling used by `Ackermann_ComputeRPM()`) was also raised
20→25 RPM, `config.h`'s `MOTOR_RPM_MAX` corrected 400→500 against a field-
measured true motor max (the 400 figure was the datasheet spec, not
reality).

**Front steering actuator — bang-bang beyond a threshold, PID inside it.**
The actuator's control (§8.2/main.c step 6) was pure bang-bang from the
project's start — 100% speed toward the target until within
`STEER_DEADBAND_DEG`, then stop — which turned out to produce a real,
field-measured **~4.3-4.5° approach-direction hysteresis**: settling to
commanded-zero from full-left-lock vs. full-right-lock left the wheels
measurably different physical distances apart, because an abrupt full-speed
stop overshoots by an amount that depends on momentum/approach direction,
not a fixed point. Now: bang-bang (100%) while `|error| > STEER_PROP_ZONE_DEG`
(4°), then a **PID** (`steer_pid.c` — present since early in the project but
never wired in until now; `SteerPID_Init()` was called at boot but
`SteerPID_Update()` was dead code) takes over inside that zone, floored at
`ACT_MIN_DUTY_PCT` (40%, a field-tuning starting guess — this actuator, a
PA-12-300-1500 300mm/7mm-s/12V unit, has no published minimum-moving-duty
spec, same situation `THR_FWD_MIN_DAC` was in before it was field-measured).
`STEER_DEADBAND_DEG` was narrowed 2.5°→1.5° accordingly.

The "centered" (stop) decision also changed for the specific case of
returning to straight (`|target_steer_deg| < CENTER_TARGET_EPS`, 1.0°): it
now requires **both** `angle_L` and `angle_R` individually within
`STEER_DEADBAND_DEG`, not just their average `steer.delta` — a plain
average can read "centered" while the two wheels individually disagree,
which is exactly how a real L/R potentiometer-calibration mismatch (§14.11)
was hiding behind a superficially-fine `delta` value. Off-center targets
keep the original delta-only check, since `angle_L != angle_R` is *expected*
there (true Ackermann inner/outer divergence during an actual turn) —
`Ackermann_ComputeRPM()`/the electronic differential is unaffected either
way, still driven by `steer.delta`. Because the per-wheel check is only
satisfiable at all if the true `angle_L`/`angle_R` mismatch is under
`2 * STEER_DEADBAND_DEG`, and this actuator is rated for only **10% duty
cycle** (also found via the datasheet search — not previously documented
anywhere in this project), a watchdog (`STEER_CENTER_WATCHDOG_US`, 2.5s)
falls back to the always-satisfiable delta-only check if the per-wheel
condition hasn't been met in time, so a drifted/failing sensor can't make
the actuator hunt indefinitely against its own duty-cycle rating.

**Status: source-complete, not yet hardware-verified.** Both changes above
are committed but await the user's next STM32CubeIDE rebuild + reflash and
field retest.

### 14.9 Verification Performed

Confirmed on hardware, rover on a jack: `check_stm32_link.py` (RPi side) shows
a steady **20.0 frames/s, zero checksum failures**, correct `ARMED`/`RC_OK`
status bits. `read_encoders.py` shows live, sane `rpm_L`/`rpm_R` (0.00,
correct with wheels off the ground) and steering angles. An `ESP32_uart_sniffer`
(a read-only tap directly on STM32 `PA2`, deliberately one-way — same safety
rule as the existing PA9 debug bridge in §2.7, never connect its TX to `PA3`
while the RPi is also wired there) was used during bring-up to isolate the
STM32 side of the link from the RPi side and prove the STM32 was transmitting
correctly the whole time the RPi-side symptom was being debugged.

Since then, §14.7's full guidance stack has run multiple real straight-line
field missions end to end (SWB flipped to AUTO, `mission.py` driving via
this same link) — tracking itself (cross-track error) has been consistently
good, **under ~7cm over a 20m run**, once the issues in §14.11 stopped
masking it.

### 14.10 File Reference

| File | Purpose |
|---|---|
| `Rover_closed_loop/Core/Src/rpi_link.c` `.h` | USART2 DMA+IDLE framing, command parse, feedback send |
| `Rover_closed_loop/Core/Src/ackermann.c` — `Ackermann_RunAuto()` | Autonomous motion pipeline: takes the RPi's absolute steering/speed targets directly, bypassing the RC stick deadband/curve; electronic differential still runs off the *measured* angle, same as manual mode |
| `Rover_closed_loop/Core/Src/wheel_pid.c` `.h` | Closed-loop forward wheel-speed PI (§14.8) — encoder-fed, replaces the old open-loop DAC guess for forward driving only |
| `Rover_closed_loop/Core/Src/steer_pid.c` `.h` | Steering actuator PID (§14.8) — now live inside `STEER_PROP_ZONE_DEG`, was dead code (`Update()` never called) for most of the project |
| `RPi_companion/uart_link.py` | RPi-side counterpart to `rpi_link.c` — same frame formats, independently maintained (no shared schema file) |
| `RPi_companion/check_stm32_link.py` | Connectivity diagnostic — frame rate, checksum failures, status bits |
| `RPi_companion/read_encoders.py` | Passive live readout of wheel RPM + steering angle |
| `RPi_companion/calibrate_steering_pots.py` | Field recalibration tool for `ADC_L/R_MIN/CENTER/MAX_RAW` (§14.7, §14.11) |
| `RPi_companion/imu.py` + `test_imu.py` + `calibrate_imu.py` | DFRobot Fermion BNO055 driver (RPi-local, I2C1 on `GPIO2`/`GPIO3`/Pin3/Pin5) |
| `RPi_companion/odometry.py`, `guidance.py`, `waypoints.py`, `mission.py`, `rover_config.py` | The guidance/motion stack — see §14.7 |
| `RPi_companion/make_straight_path.py`, `make_rectangle_path.py`, `make_circle_path.py`, `make_lawnmower_path.py`, `path_prompts.py` | Interactive per-shape path generators over `waypoints.py`, saving to `paths/<shape>_<dims>.csv` |
| `ESP32_uart_sniffer/ESP32_uart_sniffer.ino` | Bring-up diagnostic only — read-only tap on STM32 `PA2` |

### 14.11 Known Open Issues ⚠️

Full diagnostic trail for all of these — the actual field logs, numbers, and
reasoning — lives in `scratchpad.md`; this is the condensed pointer.

- **Intermittent heading-lock bias.** The BNO055's magnetometer never
  calibrates on this chassis (`imu_calib_mag`/`imu_calib_sys` read 0 in every
  field log collected — plausibly interference from the nearby drive
  motors/DAC), yet its default NDOF fusion mode still blends that reading
  into `heading_deg`. Since §14.7's heading reference locks once per
  `mission.py` run, a bad lock steers the *entire* subsequent mission one
  way (cross-track error pinned to one sign, steering command one sign, for
  the whole run) rather than centering on straight. Mitigated (not fully
  fixed) by averaging `HEADING_LOCK_SAMPLES` readings instead of trusting
  one instant — reduces but can't eliminate a sustained bias present for the
  whole averaging window. Next options if it recurs: switch the BNO055 to
  IMUPLUS mode (drops the magnetometer from fusion entirely, trading
  absolute-compass heading for gyro-integration drift immunity to motor
  interference), or physically relocate the IMU away from the motors/DAC.
- **Steering potentiometer L/R calibration.** `angle_L`/`angle_R` disagree
  by ~0.6-1.6° even at commanded-straight (where true Ackermann geometry
  says they should match exactly, since inner/outer divergence only applies
  mid-turn) — most likely because the prior pot-replacement recalibration
  judged "straight" separately per wheel instead of against one shared
  physical reference for both simultaneously. `calibrate_steering_pots.py`
  (§14.7) exists to re-measure this correctly; two field attempts have been
  run, but new values haven't been committed to `ackermann_config.h` yet —
  the actuator hysteresis below was making "center" hard to measure
  repeatably (two calibration attempts disagreed on it by ~2.9°), so that
  was fixed first (§14.8). Re-run the recalibration now that centering
  should be far more repeatable.
- **Actuator approach-direction hysteresis — fix implemented, not yet
  field-verified.** See §14.8's bang-bang/PID redesign. Expected to shrink
  the ~4.3-4.5° hysteresis substantially; awaiting the user's rebuild +
  reflash + retest to confirm.
