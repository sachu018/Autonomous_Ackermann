#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/* Call once after HAL peripheral init. Sets all outputs LOW (safe state). */
void Motor_Init(void);

/* Close / open the main contactor relay. */
void Motor_Energize(void);
void Motor_Deenergize(void);

/* Set both wheels. rpm_L/R carry sign (positive=fwd, negative=rev).
 * dac_L/R are the pre-computed throttle values (0–4095).                 */
void Motor_SetSpeeds(float rpm_L, float rpm_R, uint16_t dac_L, uint16_t dac_R);

/* Active brake: DAC → 0, BRAKE GPIOs HIGH. */
void Motor_Brake(void);

/* Coast stop: DAC → 0, all GPIOs LOW. Used when disarmed or no signal. */
void Motor_Stop(void);

/* Brake for REVERSE_BRAKE_US, then call SetSpeeds. Used on first reverse tick. */
void Motor_BrakeThenReverse(float rpm_L, float rpm_R, uint16_t dac_L, uint16_t dac_R);

/* State queries used by the main loop. */
uint8_t Motor_IsArmed(void);
uint8_t Motor_WasReversing(void);
void    Motor_SetWasReversing(uint8_t val);

#endif /* MOTOR_H */
