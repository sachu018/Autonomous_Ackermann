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
/* Narrowed 2.5 -> 1.5 deg once the actuator control stopped being pure
 * bang-bang (see STEER_PROP_ZONE_DEG below) — the old value was sized to
 * tolerate bang-bang's abrupt full-speed-until-stop overshoot; the PID
 * zone approaches gently enough that a tighter tolerance is safe to try.
 * Field-verify: if the actuator ever hunts/oscillates right at this
 * boundary, widen it back up before anything else. */
#define STEER_DEADBAND_DEG      1.5f    /* Angular error deadband     (deg)  */
                                        /* (Within ±1.5°, actuator stops/holds) */

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
 * chassis, and now genuinely live: main.c's actuator control (step 6) uses
 * bang-bang beyond STEER_PROP_ZONE_DEG, then hands off to SteerPID_Update()
 * inside it, replacing the old pure-bang-bang-to-stop behavior that
 * produced the field-measured ~4.3-4.5 deg approach-direction hysteresis
 * (see scratchpad.md). These gains were ported from the BBB project and
 * have NEVER been exercised on this hardware before — expect to retune in
 * the field, same as every other gain in this project so far. */
#define KP_STEER                5.0f
#define KI_STEER                0.1f
#define KD_STEER                0.5f

/* Integral anti-windup: clamp so the integral term alone cannot exceed this
 * many percent of duty cycle. */
#define STEER_INTEGRAL_MAX_PCT  50.0f

/* ── Actuator two-zone control (main.c step 6) ──────────────────────────────
 * |error| > STEER_PROP_ZONE_DEG: bang-bang at 100% (unchanged, fast approach).
 * |error| <= STEER_PROP_ZONE_DEG: hand off to steer_pid.c instead of running
 * bang-bang all the way to the deadband — the abrupt full-speed stop is what
 * produced the measured hysteresis. */
#define STEER_PROP_ZONE_DEG     4.0f

/* Floor on the PID's commanded duty magnitude, same role as THR_FWD_MIN_DAC
 * plays for the rear wheels: a small computed duty can be too weak to break
 * the actuator's own static friction, stalling short of centered instead of
 * creeping the last bit in.
 *
 * Was 40.0f — field-tested and found to cause a persistent wobble/limit-
 * cycle right at the zero position: with STEER_DEADBAND_DEG this tight
 * (1.5 deg), flooring ANY nonzero correction up to 40% duty was enough to
 * overshoot straight back out the other side of the deadband every time,
 * triggering the same correction in reverse — an oscillation, not the
 * gentle creep-to-stop this floor was meant to guarantee. Lowered to 15.0f
 * as a smaller first guess. STILL NOT A MEASURED VALUE — this actuator
 * (PA-12-300-1500, 300mm/7mm-s/12V, self-locking screw) has no published
 * minimum-moving-duty spec; keep field-testing and correcting, the same
 * way THR_FWD_MIN_DAC's original guess needed two rounds of correction
 * from real breakaway data before it was right.
 *
 * Wobble persisted at 15% too. User's read: opposite direction from the
 * first correction's theory — 15% may be BELOW this actuator's real
 * breakaway torque (self-locking screw drive), so it just chatters/buzzes
 * in place rather than cleanly moving and settling, instead of 40%
 * overshooting the deadband. Raised to 70.0f to test that directly.
 *
 * RESULT: wobble got WORSE at 70%, not better — confirms the ORIGINAL
 * overshoot theory (any floor this size relative to a 1.5 deg deadband
 * overshoots straight back out the other side every correction), not the
 * insufficient-torque theory. Floor removed entirely (0.0f) — the
 * `fabsf(duty) < ACT_MIN_DUTY_PCT` check in the caller (main.c step 6)
 * becomes unsatisfiable at 0.0f (fabsf() is never negative), so this is a
 * true no-op: the PID's raw output drives the actuator directly, however
 * small, with nothing forcing it up to a floor at all.
 *
 * ⚠️ Expected trade-off: very close to target, the PID's own output can
 * legitimately be smaller than this actuator's real static friction and
 * fail to move it at all — the ORIGINAL reason a floor was added in the
 * first place. If the actuator now stalls a bit short of the deadband
 * instead of wobbling, that's this trade-off showing up, not a new bug —
 * the STEER_CENTER_WATCHDOG_US (2.5s) fallback to the plain delta check
 * exists to bound exactly that case so it doesn't hang forever. If it
 * still doesn't settle acceptably, the next lever is KP_STEER (currently
 * 5.0, untuned BBB-ported gain) — a smaller Kp reaching the same duty
 * over a wider error band, rather than a floor forcing a jump, may be
 * the more correct fix than any floor value. */
#define ACT_MIN_DUTY_PCT        0.0f

/* How close |target_steer_deg| must be to 0 before the "centered" check
 * requires BOTH angle_L and angle_R individually within STEER_DEADBAND_DEG,
 * instead of just their average (steer.delta) — see step 6's comment in
 * main.c for why the average alone can hide a real per-wheel disagreement.
 * Loose enough to cover Pure Pursuit's small near-straight corrections,
 * tight enough to stay clear of genuine small turns (where angle_L/angle_R
 * legitimately diverge by Ackermann geometry, and requiring both to match
 * a single target would be wrong). */
#define CENTER_TARGET_EPS       1.0f

/* Safety bound on the per-wheel centering check above: it's only
 * satisfiable if the actual angle_L/angle_R mismatch is under
 * 2*STEER_DEADBAND_DEG. If that ever grows (pot drift, a failing sensor),
 * the PID could hunt indefinitely chasing an unreachable state — and this
 * actuator is only rated for 10% duty cycle (see scratchpad.md), so
 * sustained hunting risks cooking it. After this many microseconds of
 * continuously trying, main.c falls back to the plain delta-based check
 * (always satisfiable) so centering is guaranteed to terminate either way. */
#define STEER_CENTER_WATCHDOG_US 2500000U   /* 2.5 s */

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
/* FORWARD driving now uses a closed-loop PI (wheel_pid.c) instead of an
 * open-loop RPM->DAC guess — see that file's header comment for why. Two
 * things were tried and abandoned before landing on this:
 *   1. THR_FWD_MIN_DAC was briefly raised to 1624 (computed from a
 *      field-measured 3.5 RPM breakaway point run through the OLD open-loop
 *      formula), on the theory that the floor alone explained a ~2.8-3.2x
 *      actual-vs-commanded speed mismatch. That made the ratio WORSE
 *      (3.4-5.5x) — raising the floor while holding the far endpoint
 *      (WHEEL_RPM_MAX -> DAC_MAX) fixed lifts the WHOLE line, not just the
 *      low end.
 *   2. Pooling real (DAC, measured RPM) pairs from field logs showed the
 *      real relationship reaches WHEEL_RPM_MAX (20) at DAC~2171 (53% duty),
 *      not DAC_MAX — a genuine SLOPE error, not just a floor error, and the
 *      curve isn't perfectly linear either. A two-point linear re-fit was
 *      considered, but a static curve of any shape can't track terrain/
 *      incline/load changes in the field the way a closed loop can.
 * THR_FWD_MIN_DAC now only needs to be a REASONABLE STARTING FLOOR the PI's
 * integral term corrects away from — reverted to 1100 (close to what the
 * field data implies the true breakaway point actually is, ~1162). See
 * scratchpad.md for the full trail. */
#define THR_FWD_MIN_DAC         1100U   /* forward PI starting floor (not precision-critical) */
#define THR_REV_MIN_DAC         1810U   /* reverse stiction (~2.21 V)        */
#define THR_REV_MAX_DAC         2450U   /* reverse ceiling                   */
#define THR_REV_RPM_CAP         10.0f   /* reverse speed cap (wheel RPM)     */

/* ── Forward wheel-speed PI (wheel_pid.c) — gains carried over directly from
 * Old_files/dev_bak/ugv_pid_waypoint.py's BBBHardware, same drivetrain. */
#define KP_WHEEL                50.0f
#define KI_WHEEL                15.0f
#define WHEEL_PID_MAX_INTEGRAL  150.0f   /* clamp on the raw integral accumulator */
#define WHEEL_PID_MIN_CORRECTION (-250.0f) /* lets DAC dip below the floor if overspeeding */

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
