#ifndef IBUS_H
#define IBUS_H

#include <stdint.h>

typedef struct {
    float   Xn;            /* Steering  −1.0 … +1.0 */
    float   Yn;            /* Throttle  −1.0 … +1.0 */
    float   fwd_speed_pct; /* Forward speed cap 0.0–1.0 */
    float   rev_speed_pct; /* Fixed 1.0 as of Phase 5 — SWB no longer sets this
                             * independently, reverse uses the same cap as
                             * forward. Field kept so Ackermann_Run()'s
                             * signature didn't need to change. See config.h
                             * SWB section and scratchpad.md. */
    uint8_t swd_on;        /* 1 = arm switch ON */
    uint8_t auto_mode;     /* 1 = SWB selects AUTO (RPi UART) command source,
                             * 0 = MANUAL (RC sticks). Only takes effect while
                             * armed; see main.c. */
    uint8_t rc_ok;         /* 1 = signal present and fresh */
} IBUSData_t;

void IBUS_Init(void);
void IBUS_Read(IBUSData_t *out);
void IBUS_IdleCallback(void);  /* call from USART1_IRQHandler on IDLE flag */

#endif /* IBUS_H */
