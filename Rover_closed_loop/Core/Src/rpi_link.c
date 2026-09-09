/* ============================================================================
 *  rpi_link.c — UART link to the Raspberry Pi autonomous companion computer
 * ============================================================================
 *
 *  Command frame, RPi -> STM32, 8 bytes, little-endian:
 *      [0]     0xAA                    header
 *      [1]     0x55                    header
 *      [2..3]  int16  steer_target     degrees x100  (-4500..+4500)
 *      [4..5]  int16  speed_target     mm/s, signed
 *      [6]     uint8  seq              rolling counter (not yet consumed —
 *                                       reserved for detecting a stalled-but-
 *                                       connected RPi; see scratchpad.md)
 *      [7]     uint8  checksum         XOR of bytes [0..6]
 *
 *  Feedback frame, STM32 -> RPi, 12 bytes, little-endian:
 *      [0]     0xBB                    header
 *      [1]     0x66                    header
 *      [2..3]  int16  angle_L          degrees x100
 *      [4..5]  int16  angle_R          degrees x100
 *      [6..7]  int16  rpm_L            RPM x10
 *      [8..9]  int16  rpm_R            RPM x10
 *      [10]    uint8  status           RPI_STATUS_* bitfield
 *      [11]    uint8  checksum         XOR of bytes [0..10]
 * ============================================================================
 */

#include "rpi_link.h"
#include "config.h"
#include "main.h"
#include <stdint.h>

/* ── External HAL handles ────────────────────────────────────────────────── */
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef  htim5;

/* ── DMA receive buffer ──────────────────────────────────────────────────── */
static uint8_t dma_buf[RPI_CMD_FRAME_LEN];

/* ── Latest decoded state (written by IDLE ISR, read by main loop) ───────── */
static volatile RPiCmd_t  latest;
static volatile uint32_t  last_good_ts = 0U;

/* ── Private helpers ─────────────────────────────────────────────────────── */

static uint8_t _Checksum(const uint8_t *buf, uint8_t len)
{
    uint8_t x = 0U;
    for (uint8_t i = 0U; i < len; i++) x ^= buf[i];
    return x;
}

static uint8_t _ParseCmdFrame(const uint8_t *buf, int16_t *steer_i16, int16_t *speed_i16)
{
    if (buf[0] != RPI_CMD_HEADER_0 || buf[1] != RPI_CMD_HEADER_1) return 0U;

    uint8_t calc = _Checksum(buf, RPI_CMD_FRAME_LEN - 1U);
    if (calc != buf[RPI_CMD_FRAME_LEN - 1U]) return 0U;

    *steer_i16 = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
    *speed_i16 = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));
    return 1U;
}

/* Direct register TX — bypasses HAL_UART_Transmit()'s __HAL_LOCK(huart2), for
 * the same reason main.c's _DbgPutc() bypasses it on USART1: taking that lock
 * for a blocking send would make HAL_UART_Receive_DMA() inside
 * RPiLink_IdleCallback() return HAL_BUSY if the IDLE interrupt fires mid-send,
 * dropping the RX DMA restart and losing command frames. TX and RX are
 * independent hardware paths inside the USART, so a raw register write is
 * safe and never touches the lock. */
static void _TxByte(uint8_t b)
{
    while ((USART2->SR & USART_SR_TXE) == 0U) { }
    USART2->DR = b;
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void RPiLink_Init(void)
{
    latest.steer_target_deg = 0.0f;
    latest.speed_target_ms  = 0.0f;
    latest.link_ok           = 0U;
    last_good_ts              = 0U;

    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart2, dma_buf, RPI_CMD_FRAME_LEN);
}

void RPiLink_Read(RPiCmd_t *out)
{
    uint32_t elapsed = TIM5->CNT - last_good_ts;

    __disable_irq();
    if (last_good_ts == 0U || elapsed >= AUTO_UART_TIMEOUT_US)
        latest.link_ok = 0U;
    *out = latest;
    __enable_irq();
}

void RPiLink_IdleCallback(void)
{
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);
    HAL_UART_DMAStop(&huart2);

    int16_t steer_i16, speed_i16;
    if (_ParseCmdFrame(dma_buf, &steer_i16, &speed_i16))
    {
        __disable_irq();
        latest.steer_target_deg = (float)steer_i16 / 100.0f;
        latest.speed_target_ms  = (float)speed_i16 / 1000.0f;
        latest.link_ok           = 1U;
        last_good_ts              = TIM5->CNT;
        __enable_irq();
    }

    HAL_UART_Receive_DMA(&huart2, dma_buf, RPI_CMD_FRAME_LEN);
}

void RPiLink_SendFeedback(float angle_L_deg, float angle_R_deg,
                          float rpm_L, float rpm_R, uint8_t status)
{
    int16_t aL = (int16_t)(angle_L_deg * 100.0f);
    int16_t aR = (int16_t)(angle_R_deg * 100.0f);
    int16_t rL = (int16_t)(rpm_L * 10.0f);
    int16_t rR = (int16_t)(rpm_R * 10.0f);

    uint8_t buf[RPI_FEEDBACK_FRAME_LEN];
    buf[0]  = RPI_FEEDBACK_HEADER_0;
    buf[1]  = RPI_FEEDBACK_HEADER_1;
    buf[2]  = (uint8_t)(aL & 0xFF);
    buf[3]  = (uint8_t)((aL >> 8) & 0xFF);
    buf[4]  = (uint8_t)(aR & 0xFF);
    buf[5]  = (uint8_t)((aR >> 8) & 0xFF);
    buf[6]  = (uint8_t)(rL & 0xFF);
    buf[7]  = (uint8_t)((rL >> 8) & 0xFF);
    buf[8]  = (uint8_t)(rR & 0xFF);
    buf[9]  = (uint8_t)((rR >> 8) & 0xFF);
    buf[10] = status;
    buf[11] = _Checksum(buf, RPI_FEEDBACK_FRAME_LEN - 1U);

    for (uint8_t i = 0U; i < RPI_FEEDBACK_FRAME_LEN; i++)
        _TxByte(buf[i]);
}
