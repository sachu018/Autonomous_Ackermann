/* ============================================================================
 *  ackermann.h — Ackermann motion pipeline + electronic differential
 * ============================================================================
 */

#ifndef ACKERMANN_H
#define ACKERMANN_H

#include <stdint.h>

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

/* Full Ackermann motion pipeline for one control cycle */
void Ackermann_Run(float Xn, float Yn,
                   float fwd_pct, float rev_pct,
                   float delta_actual_deg,
                   uint8_t rc_ok,
                   AckResult_t *out);

/* Autonomous motion pipeline (Phase 5 — RPi UART link): the RPi supplies an
 * absolute physical target (steering angle in degrees, linear speed in m/s)
 * instead of joystick Xn/Yn — bypasses the stick deadband/curve shaping in
 * Ackermann_Run() entirely, since these targets are not from a human hand.
 * The electronic differential still runs off the MEASURED steering angle,
 * exactly as in manual mode — only the target source differs. */
void Ackermann_RunAuto(float steer_target_deg, float speed_target_ms,
                       float delta_actual_deg,
                       AckResult_t *out);

#endif /* ACKERMANN_H */
