/* ============================================================================
 *  ackermann.h — Ackermann motion pipeline + electronic differential
 * ============================================================================
 */

#ifndef ACKERMANN_H
#define ACKERMANN_H

#include <stdint.h>
#include "wheel_pid.h"

typedef enum {
    ACK_STATE_BRAKE = 0,
    ACK_STATE_FWD,
    ACK_STATE_FWD_LEFT,
    ACK_STATE_FWD_RIGHT,
    ACK_STATE_REV,
    ACK_STATE_REV_LEFT,
    ACK_STATE_REV_RIGHT,
    ACK_STATE_PIVOT_LEFT,
    ACK_STATE_PIVOT_RIGHT,
    ACK_STATE_DISARMED,
    ACK_STATE_NO_SIGNAL
} AckState_t;

typedef struct {
    AckState_t state;
    float      target_steer_deg;  /* Setpoint for actuator PID (-45° to +45°) */
    float      V_base;            /* Virtual rear axle centerline speed (m/s)*/
    float      rpm_L;             /* Commanded left wheel RPM (signed)       */
    float      rpm_R;             /* Commanded right wheel RPM (signed)      */
    uint16_t   dac_L;             /* Left throttle DAC value (0-4095)        */
    uint16_t   dac_R;             /* Right throttle DAC value (0-4095)       */
} AckResult_t;

/* Electronic differential: computes wheel RPMs using rear axle radius R = Wb / tan(delta) */
void Ackermann_ComputeRPM(float V_base, float delta_actual_deg, uint8_t is_reverse,
                          float *rpm_L, float *rpm_R);

/* Full Ackermann motion pipeline for one control cycle.
 *
 * meas_rpm_L/R + pid_L/R: FORWARD driving closes the throttle loop on real
 * encoder feedback (wheel_pid.c) instead of an open-loop DAC guess — see
 * ackermann_config.h's THR_FWD_MIN_DAC comment for why. meas_rpm_L/R should
 * be the current Encoder_GetRPM_L()/R() reading (previous tick's value,
 * completely normal for a discrete control loop); pid_L/pid_R are owned by
 * the caller (main.c, like steer_pid is) so their integral state persists
 * across calls. REVERSE driving is unaffected — still the original
 * open-loop _ToThrottle() path. */
void Ackermann_Run(float Xn, float Yn,
                   float fwd_pct, float rev_pct,
                   float delta_actual_deg,
                   uint8_t rc_ok,
                   float meas_rpm_L, float meas_rpm_R, float dt_s,
                   WheelPID_t *pid_L, WheelPID_t *pid_R,
                   AckResult_t *out);

/* Autonomous motion pipeline (Phase 5 — RPi UART link): the RPi supplies an
 * absolute physical target (steering angle in degrees, linear speed in m/s)
 * instead of joystick Xn/Yn — bypasses the stick deadband/curve shaping in
 * Ackermann_Run() entirely, since these targets are not from a human hand.
 * The electronic differential still runs off the MEASURED steering angle,
 * exactly as in manual mode — only the target source differs.
 * meas_rpm_L/R, dt_s, pid_L/pid_R: same closed-loop forward throttle as
 * Ackermann_Run() above — see its comment. */
void Ackermann_RunAuto(float steer_target_deg, float speed_target_ms,
                       float delta_actual_deg,
                       float meas_rpm_L, float meas_rpm_R, float dt_s,
                       WheelPID_t *pid_L, WheelPID_t *pid_R,
                       AckResult_t *out);

#endif /* ACKERMANN_H */
