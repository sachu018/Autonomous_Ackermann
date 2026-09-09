/* ============================================================================
 *  ackermann.c — Ackermann motion pipeline + electronic differential
 * ============================================================================
 */

#include "ackermann.h"
#include "ackermann_config.h"
#include "config.h"
#include <math.h>
#include <stdint.h>

/* ── Throttle mapping (stiction compensated) ────────────────────────────── */
static uint16_t _ToThrottle(float rpm, uint8_t is_reverse)
{
    float abs_rpm = fabsf(rpm);
    if (abs_rpm < 0.01f) return 0U;

    if (is_reverse)
    {
        float clamped = (abs_rpm > THR_REV_RPM_CAP) ? THR_REV_RPM_CAP : abs_rpm;
        uint32_t val = THR_REV_MIN_DAC +
                       (uint32_t)((clamped / THR_REV_RPM_CAP) *
                                  (float)(THR_REV_MAX_DAC - THR_REV_MIN_DAC));
        if (val < THR_REV_MIN_DAC) val = THR_REV_MIN_DAC;
        if (val > THR_REV_MAX_DAC) val = THR_REV_MAX_DAC;
        return (uint16_t)val;
    }
    else
    {
        uint32_t val = THR_FWD_MIN_DAC +
                       (uint32_t)((abs_rpm / WHEEL_RPM_MAX) *
                                  (float)(DAC_MAX - THR_FWD_MIN_DAC));
        if (val < THR_FWD_MIN_DAC) val = THR_FWD_MIN_DAC;
        if (val > DAC_MAX)         val = DAC_MAX;
        return (uint16_t)val;
    }
}

/* ── Electronic differential ────────────────────────────────────────────── */

void Ackermann_ComputeRPM(float V_base, float delta_actual_deg, uint8_t is_reverse,
                          float *rpm_L, float *rpm_R)
{
    float V_left, V_right;

    if (fabsf(delta_actual_deg) < ACK_STRAIGHT_DEG)
    {
        /* Straight-ahead (also guards against tan(0) singularity) */
        V_left  = V_base;
        V_right = V_base;
    }
    else
    {
        float delta_rad = delta_actual_deg * (float)M_PI / 180.0f;

        /* Turning radius about the rear axle center: R_rear = Wb / tan(delta)
         * delta > 0 (left turn)  -> R_c > 0 -> right wheel outer -> faster
         * delta < 0 (right turn) -> R_c < 0 -> left  wheel outer -> faster */
        float R_c = ACK_WHEELBASE_M / tanf(delta_rad);

        V_right = V_base * (1.0f + ACK_TRACK_M / (2.0f * R_c));
        V_left  = V_base * (1.0f - ACK_TRACK_M / (2.0f * R_c));
    }

    const float circumference = 2.0f * (float)M_PI * WHEEL_RADIUS_M;
    *rpm_L = (V_left  / circumference) * 60.0f;
    *rpm_R = (V_right / circumference) * 60.0f;

    /* Proportional scaling: scale BOTH wheels down by the same factor if either
     * exceeds the active RPM ceiling (WHEEL_RPM_MAX in fwd, THR_REV_RPM_CAP in rev).
     * This preserves the kinematic differential ratio under all conditions. */
    float rpm_limit = is_reverse ? THR_REV_RPM_CAP : WHEEL_RPM_MAX;
    float aL   = fabsf(*rpm_L);
    float aR   = fabsf(*rpm_R);
    float peak = (aL > aR) ? aL : aR;
    if (peak > rpm_limit && peak > 0.0f)
    {
        float scale = rpm_limit / peak;
        *rpm_L *= scale;
        *rpm_R *= scale;
    }
}

/* ── Motion pipeline ────────────────────────────────────────────────────── */

void Ackermann_Run(float Xn, float Yn,
                   float fwd_pct, float rev_pct,
                   float delta_actual_deg,
                   uint8_t rc_ok,
                   AckResult_t *out)
{
    /* Safe default output */
    out->state            = ACK_STATE_BRAKE;
    out->target_steer_deg = 0.0f;
    out->V_base           = 0.0f;
    out->rpm_L            = 0.0f;
    out->rpm_R            = 0.0f;
    out->dac_L            = 0U;
    out->dac_R            = 0U;

    if (!rc_ok) return;

    float abs_x = fabsf(Xn);
    float abs_y = fabsf(Yn);

    /* ── Standstill steering / pivot mode ─────────────────────────────────
     * Throttle stick neutral, steering stick active: target is proportional
     * to Xn like normal driving (see below) — wheels steer and hold at that
     * fraction of lock, no rear drive. Drive is withheld until the front
     * wheels physically swing past 80% of MAX_STEER_ANGLE_DEG, which only
     * happens if the stick is held near its own extreme; once past that
     * threshold both rear wheels crawl forward (outer faster than inner) to
     * pivot tightly around the front lock. */
    if (abs_y < ACK_STICK_DEADBAND && abs_x > ACK_STICK_DEADBAND)
    {
        /* Proportional to stick, same mapping as normal driving (NOT snapped to
         * full lock) — a 25% stick deflection at standstill should steer to
         * 25% of MAX_STEER_ANGLE_DEG and hold there, not jump to full lock. */
        out->target_steer_deg = -Xn * MAX_STEER_ANGLE_DEG;

        uint8_t lock_achieved =
            (fabsf(delta_actual_deg) >= (PIVOT_LOCK_FRACTION * MAX_STEER_ANGLE_DEG))
            ? 1U : 0U;

        if (lock_achieved)
        {
            if (Xn < 0.0f)
            {
                /* Pivot Left: Inner Left wheel crawls, Outer Right wheel drives faster */
                out->rpm_L = PIVOT_INNER_RPM;
                out->rpm_R = PIVOT_OUTER_RPM;
                out->dac_L = _ToThrottle(out->rpm_L, 0U);
                out->dac_R = _ToThrottle(out->rpm_R, 0U);
                out->state = ACK_STATE_PIVOT_LEFT;
            }
            else
            {
                /* Pivot Right: Inner Right wheel crawls, Outer Left wheel drives faster */
                out->rpm_L = PIVOT_OUTER_RPM;
                out->rpm_R = PIVOT_INNER_RPM;
                out->dac_L = _ToThrottle(out->rpm_L, 0U);
                out->dac_R = _ToThrottle(out->rpm_R, 0U);
                out->state = ACK_STATE_PIVOT_RIGHT;
            }
        }
        else
        {
            out->V_base = 0.0f;
            out->rpm_L  = 0.0f;
            out->rpm_R  = 0.0f;
            out->dac_L  = 0U;
            out->dac_R  = 0U;
            out->state  = (Xn < 0.0f) ? ACK_STATE_PIVOT_LEFT : ACK_STATE_PIVOT_RIGHT;
        }
        return;
    }

    /* ── Neutral sticks -> brake ───────────────────────────────────────── */
    if (abs_y < ACK_STICK_DEADBAND && abs_x < ACK_STICK_DEADBAND)
    {
        out->state = ACK_STATE_BRAKE;
        return;
    }

    /* ── Normal driving ───────────────────────────────────────────────── */
    uint8_t going_fwd = (Yn > 0.0f) ? 1U : 0U;
    float   safe_pct  = going_fwd ? fwd_pct : rev_pct;

    float v_max_dir = going_fwd ? VMAX_MS : VMAX_REV_MS;
    out->V_base     = Yn * v_max_dir * safe_pct;

    /* Stick Left (Xn < 0) -> positive angle target; Stick Right -> negative */
    out->target_steer_deg = -Xn * MAX_STEER_ANGLE_DEG;

    /* Electronic differential runs off the MEASURED angle, preserving true geometry */
    Ackermann_ComputeRPM(out->V_base, delta_actual_deg, (uint8_t)(!going_fwd),
                         &out->rpm_L, &out->rpm_R);

    out->dac_L = _ToThrottle(out->rpm_L, (uint8_t)(!going_fwd));
    out->dac_R = _ToThrottle(out->rpm_R, (uint8_t)(!going_fwd));

    /* State classification based on measured wheel orientation */
    if (going_fwd)
    {
        if      (delta_actual_deg >  5.0f) out->state = ACK_STATE_FWD_LEFT;
        else if (delta_actual_deg < -5.0f) out->state = ACK_STATE_FWD_RIGHT;
        else                               out->state = ACK_STATE_FWD;
    }
    else
    {
        if      (delta_actual_deg >  5.0f) out->state = ACK_STATE_REV_LEFT;
        else if (delta_actual_deg < -5.0f) out->state = ACK_STATE_REV_RIGHT;
        else                               out->state = ACK_STATE_REV;
    }
}

/* ── Autonomous motion pipeline (Phase 5) ───────────────────────────────── */

void Ackermann_RunAuto(float steer_target_deg, float speed_target_ms,
                       float delta_actual_deg,
                       AckResult_t *out)
{
    /* Clamp to the physical steering limits — unlike Xn this is not already
     * bounded to ±1.0 by a joystick, so an out-of-range RPi command must be
     * clamped here rather than trusted. */
    if (steer_target_deg >  MAX_STEER_ANGLE_DEG) steer_target_deg =  MAX_STEER_ANGLE_DEG;
    if (steer_target_deg < -MAX_STEER_ANGLE_DEG) steer_target_deg = -MAX_STEER_ANGLE_DEG;
    out->target_steer_deg = steer_target_deg;

    if (fabsf(speed_target_ms) < AUTO_SPEED_DEADBAND_MS)
    {
        out->state  = ACK_STATE_BRAKE;
        out->V_base = 0.0f;
        out->rpm_L  = 0.0f;
        out->rpm_R  = 0.0f;
        out->dac_L  = 0U;
        out->dac_R  = 0U;
        return;
    }

    uint8_t going_fwd = (speed_target_ms > 0.0f) ? 1U : 0U;
    float   v_max_dir = going_fwd ? VMAX_MS : VMAX_REV_MS;

    out->V_base = (speed_target_ms >  v_max_dir) ?  v_max_dir :
                  (speed_target_ms < -v_max_dir) ? -v_max_dir : speed_target_ms;

    /* Electronic differential runs off the MEASURED angle, same as manual mode */
    Ackermann_ComputeRPM(out->V_base, delta_actual_deg, (uint8_t)(!going_fwd),
                         &out->rpm_L, &out->rpm_R);

    out->dac_L = _ToThrottle(out->rpm_L, (uint8_t)(!going_fwd));
    out->dac_R = _ToThrottle(out->rpm_R, (uint8_t)(!going_fwd));

    /* State classification based on measured wheel orientation — same
     * thresholds as Ackermann_Run()'s normal-driving branch. */
    if (going_fwd)
    {
        if      (delta_actual_deg >  5.0f) out->state = ACK_STATE_FWD_LEFT;
        else if (delta_actual_deg < -5.0f) out->state = ACK_STATE_FWD_RIGHT;
        else                               out->state = ACK_STATE_FWD;
    }
    else
    {
        if      (delta_actual_deg >  5.0f) out->state = ACK_STATE_REV_LEFT;
        else if (delta_actual_deg < -5.0f) out->state = ACK_STATE_REV_RIGHT;
        else                               out->state = ACK_STATE_REV;
    }
}
