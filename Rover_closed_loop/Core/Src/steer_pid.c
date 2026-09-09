/* ============================================================================
 *  steer_pid.c — PID controller for the front steering actuator
 * ============================================================================
 */

#include "steer_pid.h"
#include "ackermann_config.h"
#include "config.h"
#include <stdint.h>

void SteerPID_Init(SteerPID_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    SteerPID_Reset(pid);
}

void SteerPID_Reset(SteerPID_t *pid)
{
    pid->integral    = 0.0f;
    pid->prev_actual = 0.0f;
    pid->initialized = 0U;
}

float SteerPID_Update(SteerPID_t *pid, float target_deg, float actual_deg,
                      float dt_s, float *err_out)
{
    float error = target_deg - actual_deg;
    if (err_out != 0) *err_out = error;

    /* Guard against zero or absurd dt */
    if (dt_s <= 0.0f || dt_s > 0.5f) dt_s = 1.0f / (float)LOOP_HZ;

    /* Integral with anti-windup clamping */
    pid->integral += error * dt_s;
    if (pid->ki != 0.0f)
    {
        float lim = STEER_INTEGRAL_MAX_PCT / pid->ki;
        if (pid->integral >  lim) pid->integral =  lim;
        if (pid->integral < -lim) pid->integral = -lim;
    }
    else
    {
        pid->integral = 0.0f;
    }

    /* Derivative on measurement (prevents derivative kick on stick step changes) */
    float derivative = 0.0f;
    if (pid->initialized)
    {
        derivative = -(actual_deg - pid->prev_actual) / dt_s;
    }
    else
    {
        pid->initialized = 1U;
    }
    pid->prev_actual = actual_deg;

    float out = (pid->kp * error)
              + (pid->ki * pid->integral)
              + (pid->kd * derivative);

    if (out >  100.0f) out =  100.0f;
    if (out < -100.0f) out = -100.0f;
    return out;
}
