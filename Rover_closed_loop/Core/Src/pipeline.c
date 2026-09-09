#include "pipeline.h"
#include "kinematics.h"
#include "config.h"
#include <math.h>
#include <stdint.h>

/* Convert wheel RPM to a 12-bit DAC throttle value. */
static uint16_t _ToThrottle(float rpm)
{
    uint32_t val = (uint32_t)((fabsf(rpm) / WHEEL_RPM_MAX) * (float)DAC_MAX);
    return (uint16_t)(val > DAC_MAX ? DAC_MAX : val);
}

/* Scale V and W by the active speed cap. */
static void _ApplySafety(float *V, float *W, float safe_pct)
{
    float v_limit = VMAX_MS * safe_pct;
    if (fabsf(*V) > v_limit)
        *V = ((*V) > 0.0f) ? v_limit : -v_limit;

    /* Scale turning proportionally when moving, spot turns keep full W */
    if (fabsf(*V) > 0.1f)
        *W *= safe_pct;
}

void Pipeline_Run(float Xn, float Yn,
                  float fwd_pct, float rev_pct,
                  uint8_t rc_ok,
                  MotionResult_t *result)
{
    /* Default to safe zero output */
    result->V              = 0.0f;
    result->W              = 0.0f;
    result->rpm_L          = 0.0f;
    result->rpm_R          = 0.0f;
    result->dac_L          = 0U;
    result->dac_R          = 0U;
    result->actuator_steer = 0.0f;

    /* ── Guard: no RC or both sticks neutral ──────────────────────────────── */
    if (!rc_ok || (Yn == 0.0f && Xn == 0.0f))
    {
        result->state = STATE_BRAKE;
        return;
    }

    /* ── Pivot / spot turn (Yn = 0, Xn ≠ 0) ─────────────────────────────── */
    if (Yn == 0.0f && Xn != 0.0f)
    {
        if (Xn > 0.0f)
        {
            result->state          = STATE_SPOT_RIGHT;
            result->rpm_L          = SPOT_RPM;
            result->rpm_R          = 0.0f;
            result->actuator_steer = -1.0f; /* Retract -> turn front wheels right */
        }
        else
        {
            result->state          = STATE_SPOT_LEFT;
            result->rpm_L          = 0.0f;
            result->rpm_R          = SPOT_RPM;
            result->actuator_steer = +1.0f; /* Extend -> turn front wheels left */
        }
        result->dac_L = _ToThrottle(result->rpm_L);
        result->dac_R = _ToThrottle(result->rpm_R);
        return;
    }

    /* ── Differential drive ──────────────────────────────────────────────── */
    uint8_t going_fwd = (Yn > 0.0f) ? 1U : 0U;
    float   safe_pct  = going_fwd ? fwd_pct : rev_pct;

    float V = Yn * VMAX_MS;

    /* Cubic steering curve: reduces abrupt inner-wheel deceleration at speed */
    float abs_xn   = fabsf(Xn);
    float abs_yn   = fabsf(Yn);
    float w_scale  = (abs_xn * abs_xn * abs_xn) / (abs_yn + EPSILON) * EPSILON;
    float W        = (Xn < 0.0f ? w_scale : -w_scale) * WMAX_RADS;

    /* Reverse: flip steering sense to match driver expectation */
    if (!going_fwd) W = -W;

    _ApplySafety(&V, &W, safe_pct);

    float rpm_L, rpm_R;
    Kinematics_ComputeRPM(V, W, &rpm_L, &rpm_R);

    result->V              = V;
    result->W              = W;
    result->rpm_L          = rpm_L;
    result->rpm_R          = rpm_R;
    result->dac_L          = _ToThrottle(rpm_L);
    result->dac_R          = _ToThrottle(rpm_R);
    result->actuator_steer = -Xn; /* Xn < 0 (Left) -> +mag (Extend), Xn > 0 (Right) -> -mag (Retract) */

    /* Determine state label */
    if (going_fwd)
    {
        if      (Xn >  0.05f) result->state = STATE_FWD_RIGHT;
        else if (Xn < -0.05f) result->state = STATE_FWD_LEFT;
        else                   result->state = STATE_FWD;
    }
    else
    {
        if      (Xn >  0.05f) result->state = STATE_REV_RIGHT;
        else if (Xn < -0.05f) result->state = STATE_REV_LEFT;
        else                   result->state = STATE_REV;
    }
}
