/* ============================================================================
 *  ackermann_config.h — Ackermann front-steering configuration
 * ============================================================================
 *
 *  This header is ADDITIVE. It does not replace Core/Inc/config.h — the
 *  existing rover geometry, iBUS map, DAC addresses and timing constants all
 *  still come from there. Include BOTH.
 *
 *  ── SIGN CONVENTION (memorise this; every module below depends on it) ──
 *
 *      LEFT turn  = POSITIVE steering angle (+)
 *      RIGHT turn = NEGATIVE steering angle (-)
 *
 *      Joystick Xn > 0 (stick RIGHT) → target_steer NEGATIVE → right turn
 *      Joystick Xn < 0 (stick LEFT)  → target_steer POSITIVE → left  turn
 *
 *      ADC: ADC_*_MIN_RAW corresponds to full RIGHT lock (-MAX_STEER_ANGLE)
 *           ADC_*_MAX_RAW corresponds to full LEFT  lock (+MAX_STEER_ANGLE)
 *
 *      Kinematics: delta > 0 (left turn) → right wheel is OUTER → faster.
 * ============================================================================
 */

#ifndef ACKERMANN_CONFIG_H
#define ACKERMANN_CONFIG_H

#include "config.h"
#include <math.h>

#ifndef M_PI
#define M_PI                    3.14159265358979323846f
#endif

/* ── Chassis geometry ───────────────────────────────────────────────────── */
#define ACK_WHEELBASE_M         0.80f   /* Wb — front axle to rear axle (m)  */
#define ACK_TRACK_M             0.65f   /* Lt — left to right wheel      (m) */

/* ── Steering limits & deadband ─────────────────────────────────────────── */
#define MAX_STEER_ANGLE_DEG     45.0f   /* Max physical steering lock (deg)  */
#define STEER_DEADBAND_DEG      2.5f    /* Angular error deadband     (deg)  */
                                        /* (Within ±2.5°, actuator stops/holds) */

/* Below this |angle| the Ackermann equations are treated as "straight" to
 * avoid the tan(delta) singularity at zero.
 * MUST be >= STEER_DEADBAND_DEG. The actuator's bang-bang control can leave
 * the wheels resting anywhere inside +-STEER_DEADBAND_DEG of a commanded
 * angle (it stops correcting once error is that small) — if this threshold
 * were narrower than the actuator's deadband, a residual angle the actuator
 * considers "close enough to straight" could still be wide enough to make
 * Ackermann_ComputeRPM() apply a real (if small) L/R speed differential,
 * causing the rover to drift consistently toward whichever side the
 * actuator happens to rest on when commanded straight. (Diagnosed on
 * hardware: persistent left drift on a "straight" command, actuator resting
 * within its old 1.0 deg window's blind spot — see scratchpad.md.)        */
#define ACK_STRAIGHT_DEG        3.0f

/* ── Steering PID gains (from the BBB config.py port — UNTUNED on this
 * chassis). steer_pid.c is currently dead code — main.c's actuator control
 * is bang-bang, not PID-driven — but SteerPID_Init() is still called at
 * boot, so these must exist and compile. */
#define KP_STEER                5.0f
#define KI_STEER                0.1f
#define KD_STEER                0.5f

/* Integral anti-windup: clamp so the integral term alone cannot exceed this
 * many percent of duty cycle. */
#define STEER_INTEGRAL_MAX_PCT  50.0f

/* ── ADS1115 steering potentiometer feedback ────────────────────────────── */
#define ADS1115_I2C_ADDR        0x48U   /* 7-bit. Shares I2C1 with DACs 0x60/0x61 */

/* Raw 16-bit ADC values at the mechanical stops.
 * Re-calibrated after BOTH potentiometers were replaced (see scratchpad.md).
 * Spans are now well-balanced: L 4426-3096=1330 / 5793-4426=1367 (~3% split),
 * R 3167-2124=1043 / 4160-3167=993 (~5% split) — a healthy result, unlike
 * the earlier replacement's 3.1x L/R span mismatch this note used to warn
 * about (that warning no longer applies to these pots).                    */
#define ADC_L_MIN_RAW           3096    /* Left  pot @ full RIGHT lock       */
#define ADC_L_MAX_RAW           5793    /* Left  pot @ full LEFT  lock       */
#define ADC_R_MIN_RAW           2124    /* Right pot @ full RIGHT lock       */
#define ADC_R_MAX_RAW           4160    /* Right pot @ full LEFT  lock       */

/* Raw ADC value at true straight-ahead, per wheel. Used as the pivot point
 * of a 3-point (piecewise-linear) calibration: MIN_RAW->CENTER_RAW maps
 * linearly to -MAX_STEER_ANGLE_DEG..0, CENTER_RAW->MAX_RAW maps linearly to
 * 0..+MAX_STEER_ANGLE_DEG. This replaces an earlier flat-offset trim
 * (subtracted after a single min->max line, then clamped) which fixed the
 * zero-point but silently compressed one side's usable range: at full LEFT
 * lock it only reached ~29 deg (short of the 36 deg / 80% needed to trigger
 * the standstill pivot — this is why left pivot never engaged), while the
 * right side saturated at the clamp before reaching the true lock. The
 * piecewise map has no such asymmetry: each lock reaches exactly
 * +-MAX_STEER_ANGLE_DEG and center reads exactly 0, independent of how far
 * either lock's raw value is from center. If MIN/MAX/CENTER raw ever
 * changes again, all three must be re-measured together.                  */
#define ADC_L_CENTER_RAW         4426
#define ADC_R_CENTER_RAW         3167

/* Reject obviously-bad readings (disconnected wiper floats or rails).      */
#define ADC_SANITY_MIN          200
#define ADC_SANITY_MAX          32000

/* Consecutive failed/insane ADC reads before sensor declared dead (5 = 250ms) */
#define STEER_FAULT_THRESHOLD   5U

/* ── Cytron MD10C actuator driver ───────────────────────────────────────── */
/* PWM  → PA8  (TIM1_CH1)   speed
 * DIR  → PA5  (GPIO out)   direction                                        */
#define ACT_PWM_ARR             999U    /* TIM1 ARR register value           */

/* Mounting polarity. If steering RIGHT makes wheels turn LEFT, flip to -1   */
#define ACTUATOR_DIR            ( 1)

/* ── Rear-wheel throttle mapping (stiction compensated) ─────────────────── */
#define THR_FWD_MIN_DAC         1100U   /* forward stiction threshold        */
#define THR_REV_MIN_DAC         1810U   /* reverse stiction (~2.21 V)        */
#define THR_REV_MAX_DAC         2450U   /* reverse ceiling                   */
#define THR_REV_RPM_CAP         10.0f   /* reverse speed cap (wheel RPM)     */

/* Max reverse linear velocity derived from RPM cap and wheel geometry       */
#define VMAX_REV_MS             ((THR_REV_RPM_CAP / 60.0f) * (2.0f * (float)M_PI * WHEEL_RADIUS_M))

/* ── Standstill pivot ("spot turn" equivalent) ──────────────────────────── */
#define PIVOT_LOCK_FRACTION     0.80f   /* need 80 % of max lock first       */
#define PIVOT_OUTER_RPM         10.0f   /* outer rear wheel speed during pivot */
#define PIVOT_INNER_RPM         6.0f    /* inner rear wheel speed during pivot */

/* Stick thresholds that select pivot vs normal driving.                    */
#define ACK_STICK_DEADBAND      0.05f

/* ── Autonomous (RPi) mode ───────────────────────────────────────────────
 * Below this |speed_target_ms| commanded by the RPi over UART, treat as a
 * stop and brake rather than trying to creep — plays the same role
 * ACK_STICK_DEADBAND plays for the manual stick in Ackermann_Run(). */
#define AUTO_SPEED_DEADBAND_MS   0.03f

#endif /* ACKERMANN_CONFIG_H */
