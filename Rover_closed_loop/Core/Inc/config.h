#ifndef CONFIG_H
#define CONFIG_H

/* ── Rover Geometry ─────────────────────────────────────────────────────────── */
#define WHEEL_RADIUS_M          0.175f
#define TRACK_WIDTH_M           0.60f
#define VMAX_MS                 0.366f
#define WMAX_RADS               1.22f

/* ── Motor / Gearbox ────────────────────────────────────────────────────────── */
#define MOTOR_RPM_MAX           400.0f
#define GEAR_RATIO              20.0f
#define WHEEL_RPM_MAX           (MOTOR_RPM_MAX / GEAR_RATIO)  /* 20.0 RPM */

/* ── Encoder ────────────────────────────────────────────────────────────────── */
#define ENCODER_PPR             600U
#define ENCODER_COUNTS_REV      2400U   /* 600 PPR × 4 edges (TI12 quadrature)  */
#define ENC_STOPPED_US          500000U /* No pulse for 500ms → report 0 RPM    */

/* ── Control Tuning ─────────────────────────────────────────────────────────── */
#define DEADBAND                0.05f   /* Stick dead zone (±5% around centre)  */
#define EPSILON                 0.6f    /* Steering curve shaping factor        */
#define MIN_RATIO               0.2f    /* Inner wheel minimum speed ratio      */
#define SPOT_RPM                8.75f   /* Wheel RPM for pivot (spot) turns     */

/* ── Timing ─────────────────────────────────────────────────────────────────── */
#define LOOP_HZ                 20U
#define LOOP_PERIOD_US          50000U  /* 1 / 20 Hz = 50 ms                   */
#define CONTACTOR_DEBOUNCE_US   500000U /* 500 ms arm/disarm debounce           */
#define MOTOR_SEQ_DELAY_US      20000U  /* 20 ms between GPIO set and DAC write */
#define REVERSE_BRAKE_US        500000U /* 500 ms brake before engaging reverse */
#define DAC_RAMP_STEPS          10U     /* Steps to ramp DAC on a direction change */
#define DAC_RAMP_STEP_US        20000U  /* 20 ms/step → ~200 ms full ramp        */

/* ── DAC — MCP4725 (I2C throttle generation) ────────────────────────────────── */
#define DAC_LEFT_ADDR           0x61U   /* Left motor DAC I2C address           */
#define DAC_RIGHT_ADDR          0x60U   /* Right motor DAC I2C address          */
#define DAC_MAX                 4095U   /* 12-bit full scale → 5 V              */
#define DAC_ZERO                0U

/* ── iBUS Protocol ──────────────────────────────────────────────────────────── */
#define IBUS_BAUD               115200U
#define IBUS_FRAME_LEN          32U
#define IBUS_HEADER_0           0x20U
#define IBUS_HEADER_1           0x40U
#define IBUS_NUM_CH             10U
#define IBUS_VAL_MIN            1000U
#define IBUS_VAL_MID            1500U
#define IBUS_VAL_MAX            2000U
#define RC_TIMEOUT_US           400000U /* 400 ms without valid frame → RC lost */

/* ── iBUS Channel Map ───────────────────────────────────────────────────────── */
#define IBUS_CH_X               0U  /* Right stick left/right  — Steering      */
#define IBUS_CH_Y               1U  /* Right stick up/down     — Throttle      */
#define IBUS_CH_SWC             4U  /* 3-position switch       — Fwd speed cap */
#define IBUS_CH_SWD             5U  /* 2-position switch       — Arm / disarm  */
#define IBUS_CH_SWB             6U  /* 2-position switch — MANUAL/AUTO command
                                      * source select (was Rev speed cap; see
                                      * SWB section below and scratchpad.md)   */

/* ── SWC — Forward Speed Cap (3-position) ───────────────────────────────────── */
#define SWC_THRESH_LO           1250U
#define SWC_THRESH_HI           1750U
#define SWC_SPEED_LOW           0.30f
#define SWC_SPEED_MID           0.60f
#define SWC_SPEED_HIGH          1.00f

/* ── SWB — Manual / Auto Command-Source Select (2-position) ─────────────────────
 * Repurposed for Phase 5 (RPi autonomous integration) from its original role
 * as the reverse-speed-cap switch — see scratchpad.md "SWB Repurposed" note.
 *   raw <  SWB_THRESH -> MANUAL (RC sticks drive Ackermann_Run(), as before)
 *   raw >= SWB_THRESH -> AUTO   (RPi UART link drives Ackermann_RunAuto(),
 *                                only while RPiCmd_t.link_ok is also true —
 *                                see AUTO_UART_TIMEOUT_US below)
 * Low/untouched position defaults to MANUAL, matching the FS-i6X's typical
 * switch-down-is-off convention — the safer default if SWB is never touched.
 * Consequence: reverse-speed-cap is gone. Reverse now always runs at the same
 * 100% cap forward already could reach; there is no longer a separate
 * limiter for it. */
#define SWB_THRESH              1500U

/* ── SWD — Arm / Disarm Threshold ──────────────────────────────────────────── */
#define SWD_THRESH              1450U

/* ── RPi Autonomous Link (USART2, PA2 TX / PA3 RX) ──────────────────────────── */
#define RPI_LINK_BAUD            115200U
#define RPI_CMD_FRAME_LEN        8U      /* header(2)+steer(2)+speed(2)+seq(1)+cksum(1) */
#define RPI_CMD_HEADER_0         0xAAU
#define RPI_CMD_HEADER_1         0x55U
#define RPI_FEEDBACK_FRAME_LEN   12U     /* header(2)+angleL(2)+angleR(2)+rpmL(2)+rpmR(2)+status(1)+cksum(1) */
#define RPI_FEEDBACK_HEADER_0    0xBBU
#define RPI_FEEDBACK_HEADER_1    0x66U
/* No valid command frame within this window while SWB=AUTO -> brake, same
 * failsafe class as RC_TIMEOUT_US above. See architecture.md §3.5. */
#define AUTO_UART_TIMEOUT_US     300000U

/* ── Motor Direction Multipliers ────────────────────────────────────────────── */
/* Flip either to -1 if a wheel runs backwards relative to expected direction   */
#define LEFT_MOTOR_DIR          ( 1)
#define RIGHT_MOTOR_DIR         ( 1)

/* ── Linear Actuator Steering (MD10C on PA8 PWM / PA5 DIR) ──────────────────── */
#define ACTUATOR_PWM_MAX        999U    /* Full speed PWM (matches TIM1 ARR)   */
#define ACTUATOR_DIR_LEFT       1U      /* GPIO HIGH -> Extend -> Left turn    */
#define ACTUATOR_DIR_RIGHT      0U      /* GPIO LOW  -> Retract -> Right turn  */

#endif /* CONFIG_H */
