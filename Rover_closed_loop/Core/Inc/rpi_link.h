/* ============================================================================
 *  rpi_link.h — UART link to the Raspberry Pi autonomous companion computer
 * ============================================================================
 *  USART2, PA2 (TX) / PA3 (RX), DMA + IDLE-line framing — same pattern as
 *  ibus.c/USART1 for the RC receiver. Plays the same role for the RPi that
 *  ibus.c plays for the RC transmitter: parses fixed-size binary frames into
 *  a latest-value struct the main loop polls once per tick.
 *
 *  See architecture.md §3 for the protocol design and rationale.
 * ============================================================================
 */

#ifndef RPI_LINK_H
#define RPI_LINK_H

#include <stdint.h>

typedef struct {
    float   steer_target_deg;  /* Target steering angle, -45.0..+45.0 (AUTO mode) */
    float   speed_target_ms;   /* Target linear speed, signed m/s (AUTO mode)      */
    uint8_t link_ok;           /* 1 = a valid command frame arrived within
                                 * AUTO_UART_TIMEOUT_US of the last read           */
} RPiCmd_t;

/* Status bitfield reported to the RPi in the feedback frame. */
#define RPI_STATUS_ARMED        (1U << 0)
#define RPI_STATUS_STEER_FAULT  (1U << 1)
#define RPI_STATUS_AUTO_ACTIVE  (1U << 2)
#define RPI_STATUS_RC_OK        (1U << 3)

/* Enable IDLE IT + start RX DMA. Call once during boot, after MX_USART2_UART_Init(). */
void RPiLink_Init(void);

/* Copy out the latest parsed command. Marks link_ok = 0 if nothing valid has
 * arrived within AUTO_UART_TIMEOUT_US — call once per main loop tick. */
void RPiLink_Read(RPiCmd_t *out);

/* Call from USART2_IRQHandler() when the IDLE flag is set. */
void RPiLink_IdleCallback(void);

/* Builds and transmits one feedback frame to the RPi. Uses a direct register
 * write (bypasses HAL_UART_Transmit()'s lock) for the same reason main.c's
 * _DbgPutc() does on USART1 TX — see the comment on _TxByte() in rpi_link.c. */
void RPiLink_SendFeedback(float angle_L_deg, float angle_R_deg,
                          float rpm_L, float rpm_R, uint8_t status);

#endif /* RPI_LINK_H */
