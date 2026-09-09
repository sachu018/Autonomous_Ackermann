/* ============================================================================
 *  ads1115.h — ADS1115 external ADC, steering potentiometer feedback
 * ============================================================================
 *  Shares I2C1 (PB6/PB7) with the throttle DACs — address 0x48.
 * ============================================================================
 */

#ifndef ADS1115_H
#define ADS1115_H

#include <stdint.h>

typedef struct {
    float   angle_L;      /* Left  wheel angle, degrees. +left / -right      */
    float   angle_R;      /* Right wheel angle, degrees                      */
    float   delta;        /* Average steering angle, degrees (+left / -right)*/
    uint8_t valid;        /* 1 = usable reading, 0 = sensor failure          */
} SteerAngles_t;

/* Probe the ADC on I2C1. Returns 1 if present, 0 on failure. */
uint8_t ADS1115_Init(void);

/* Read both channels and convert to angles. Call once per 20 Hz tick.
 * Holds previous valid angle during transient I2C glitches.
 * Only flags valid = 0 after STEER_FAULT_THRESHOLD consecutive failures. */
void ADS1115_Read(SteerAngles_t *out);

/* Raw channel values for potentiometer calibration. Returns 1 on success. */
uint8_t ADS1115_ReadRaw(int16_t *raw_L, int16_t *raw_R);

/* Returns 1 if sensor health is confirmed healthy (no persistent fault). */
uint8_t ADS1115_IsHealthy(void);

#endif /* ADS1115_H */
