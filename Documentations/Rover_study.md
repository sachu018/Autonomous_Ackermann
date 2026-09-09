# Rover System — Complete Technical Reference (STM32)

> **Platform:** STM32F411CEU6 "Black Pill" — bare-metal C, STM32Cube HAL
> **Supersedes:** the original BeagleBone Black version of this document. The BBB
> system is retained only as historical/derivation context in §13.
> **Purpose:** Give a new project engineer complete understanding of the rover —
> hardware, firmware, safety, and the failure modes already discovered — so they
> can modify, extend, or debug it independently.
> **Status legend used throughout:** ✅ confirmed working on hardware ·
> ⚠️ known issue / open · 🔍 inferred, not measured

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
