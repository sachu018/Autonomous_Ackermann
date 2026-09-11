/* ============================================================================
 *  wheel_pid.h — closed-loop rear-wheel speed control (encoder-fed PI)
 * ============================================================================
 *  Replaces ackermann.c's old open-loop "guess a DAC from a target RPM"
 *  throttle mapping for FORWARD driving. That approach required knowing the
 *  true DAC->actual-RPM curve, which turned out to be badly nonlinear (see
 *  scratchpad.md) and — more fundamentally — CAN'T be a fixed constant at
 *  all, since terrain, incline, and load all shift it in the field. A
 *  closed loop that corrects DAC against the actual measured encoder RPM
 *  sidesteps needing that curve entirely.
 *
 *  Structurally mirrors steer_pid.h (this project's other PID) — same
 *  Init/Reset/Update shape. Ki/Kp starting values are carried over directly
 *  from Old_files/dev_bak/ugv_pid_waypoint.py's BBBHardware wheel-speed PI
 *  loops (Kp=50, Ki=15) — same motors, same gearbox, same DAC hardware, so
 *  those are real field-tuned gains, not a blind guess.
 *
 *  Reverse driving is NOT covered here — still uses ackermann.c's original
 *  open-loop _ToThrottle() with THR_REV_MIN_DAC/THR_REV_MAX_DAC. Deliberate
 *  scope decision (forward only, for now).
 * ============================================================================
 */

#ifndef WHEEL_PID_H
#define WHEEL_PID_H

#include <stdint.h>

typedef struct {
    float kp, ki;
    float integral;
} WheelPID_t;

void WheelPID_Init(WheelPID_t *pid, float kp, float ki);

/* Clear accumulated integral — call whenever the target speed drops to zero
 * (stopped/braking) or on disarm/RC-loss, so a stale correction from the
 * last driving segment can't cause an unwanted kick when driving resumes. */
void WheelPID_Reset(WheelPID_t *pid);

/* Step the loop. target_rpm/measured_rpm are UNSIGNED magnitudes — direction
 * is handled separately by motor.c's existing REV GPIO logic, this function
 * only ever produces a throttle magnitude. Returns a DAC value (0-DAC_MAX).
 * measured_rpm is expected to be the PREVIOUS tick's Encoder_GetRPM_L()/R()
 * value — normal one-tick-delayed feedback for a discrete control loop,
 * same as the BBB version used. */
uint16_t WheelPID_Update(WheelPID_t *pid, float target_rpm, float measured_rpm,
                         float dt_s);

#endif /* WHEEL_PID_H */
