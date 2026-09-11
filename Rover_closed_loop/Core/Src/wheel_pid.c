/* ============================================================================
 *  wheel_pid.c — closed-loop rear-wheel speed control (encoder-fed PI)
 * ============================================================================
 */

#include "wheel_pid.h"
#include "ackermann_config.h"
#include "config.h"
#include <stdint.h>

void WheelPID_Init(WheelPID_t *pid, float kp, float ki)
{
    pid->kp = kp;
    pid->ki = ki;
    WheelPID_Reset(pid);
}

void WheelPID_Reset(WheelPID_t *pid)
{
    pid->integral = 0.0f;
}

uint16_t WheelPID_Update(WheelPID_t *pid, float target_rpm, float measured_rpm,
                         float dt_s)
{
    /* Guard against zero or absurd dt — same fallback as steer_pid.c. */
    if (dt_s <= 0.0f || dt_s > 0.5f) dt_s = 1.0f / (float)LOOP_HZ;

    float error = target_rpm - measured_rpm;

    /* Integral with anti-windup clamping on the raw accumulator (not on
     * ki*integral) — mirrors the BBB PIDController's max_integral pattern. */
    pid->integral += error * dt_s;
    if (pid->integral >  WHEEL_PID_MAX_INTEGRAL) pid->integral =  WHEEL_PID_MAX_INTEGRAL;
    if (pid->integral < -WHEEL_PID_MAX_INTEGRAL) pid->integral = -WHEEL_PID_MAX_INTEGRAL;

    float correction = (pid->kp * error) + (pid->ki * pid->integral);

    /* Correction is added on top of THR_FWD_MIN_DAC (the stiction-breaking
     * floor) — clamp the correction itself first (BBB: min_output=-250,
     * max_output=DAC_MAX-THR_FWD_MIN_DAC), THEN clamp the final DAC to the
     * hardware's valid range. Two-stage clamp, same as the BBB version. */
    float max_correction = (float)(DAC_MAX - THR_FWD_MIN_DAC);
    if (correction > max_correction)          correction = max_correction;
    if (correction < WHEEL_PID_MIN_CORRECTION) correction = WHEEL_PID_MIN_CORRECTION;

    float dac_f = (float)THR_FWD_MIN_DAC + correction;
    if (dac_f < 0.0f)            dac_f = 0.0f;
    if (dac_f > (float)DAC_MAX)  dac_f = (float)DAC_MAX;

    return (uint16_t)dac_f;
}
