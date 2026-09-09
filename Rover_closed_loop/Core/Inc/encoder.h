#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/* Call once after HAL peripheral init, before the main loop. */
void  Encoder_Init(void);

/* Call every control loop tick. Reads counter deltas, updates RPM. */
void  Encoder_Update(void);

/* Return latest signed wheel RPM (positive = forward, negative = reverse). */
float Encoder_GetRPM_L(void);
float Encoder_GetRPM_R(void);

/* Raw extended counter values as last sampled by Encoder_Update(). Lets a
 * bench test confirm the encoder hardware/wiring is working even if the
 * RPM maths is in doubt — counts moving with RPM wrong is a firmware bug,
 * counts frozen is a wiring/pin problem. */
int32_t Encoder_GetCount_L(void);
int32_t Encoder_GetCount_R(void);

#endif /* ENCODER_H */
