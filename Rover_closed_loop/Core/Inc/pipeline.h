#ifndef PIPELINE_H
#define PIPELINE_H

#include <stdint.h>

typedef enum {
    STATE_BRAKE = 0,
    STATE_DISARMED,
    STATE_NO_SIGNAL,
    STATE_FWD,
    STATE_FWD_LEFT,
    STATE_FWD_RIGHT,
    STATE_REV,
    STATE_REV_LEFT,
    STATE_REV_RIGHT,
    STATE_SPOT_LEFT,
    STATE_SPOT_RIGHT
} MotionState_t;

typedef struct {
    MotionState_t state;
    float         V;       /* Linear velocity  (m/s)  */
    float         W;       /* Angular velocity (rad/s) */
    float         rpm_L;   /* Commanded left wheel RPM  (signed) */
    float         rpm_R;   /* Commanded right wheel RPM (signed) */
    uint16_t      dac_L;   /* DAC throttle value 0–4095 */
    uint16_t      dac_R;
    float         actuator_steer; /* Linear actuator steering command (-1.0 to +1.0) */
} MotionResult_t;

/*
 * Compute the motion command from normalised RC inputs.
 * Xn    : steering  −1.0 (full left) … +1.0 (full right)
 * Yn    : throttle  −1.0 (full rev)  … +1.0 (full fwd)
 * fwd_pct : forward speed cap  0.0–1.0 (from SWC switch)
 * rev_pct : reverse speed cap  0.0–1.0 (from SWB switch)
 * rc_ok   : 1 = signal present, 0 = lost
 */
void Pipeline_Run(float Xn, float Yn,
                  float fwd_pct, float rev_pct,
                  uint8_t rc_ok,
                  MotionResult_t *result);

#endif /* PIPELINE_H */
