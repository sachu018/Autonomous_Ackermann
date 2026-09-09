/* ============================================================================
 *  steer_pid.h — PID controller for the front steering actuator
 * ============================================================================
 */

#ifndef STEER_PID_H
#define STEER_PID_H

#include <stdint.h>

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_actual;
    uint8_t initialized;
} SteerPID_t;

void SteerPID_Init(SteerPID_t *pid, float kp, float ki, float kd);

/* Clear accumulated integral and state on disarm / sensor fault / RC loss */
void SteerPID_Reset(SteerPID_t *pid);

/* Step PID using Derivative-on-Measurement.
 * Returns duty cycle in [-100, +100]. POSITIVE = extend, NEGATIVE = retract. */
float SteerPID_Update(SteerPID_t *pid, float target_deg, float actual_deg,
                      float dt_s, float *err_out);

#endif /* STEER_PID_H */
