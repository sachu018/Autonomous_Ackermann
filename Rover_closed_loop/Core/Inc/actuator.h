/* ============================================================================
 *  actuator.h — Cytron MD10C linear steering actuator driver
 * ============================================================================
 *  Interface:
 *      PWM → PA8 (TIM1_CH1)   — speed (0–100 %)
 *      DIR → PA5 (GPIO out)   — direction (extend / retract)
 * ============================================================================
 */

#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <stdint.h>

/* Start PWM channel and initialize DIR LOW. Call once on startup. */
void Actuator_Init(void);

/* Drive actuator with duty cycle (0.0 to 100.0) and direction (1=extend, 0=retract). */
void Actuator_SetSpeed(float duty_pct, uint8_t extend);

/* Stop driving actuator (PWM 0%). */
void Actuator_Stop(void);

/* Last commanded duty (+ = extend, - = retract). Telemetry only. */
float Actuator_GetDuty(void);

#endif /* ACTUATOR_H */
