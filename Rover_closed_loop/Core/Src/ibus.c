#include "ibus.h"
#include "config.h"
#include "main.h"
#include <string.h>
#include <stdint.h>

/* ── External HAL handles ────────────────────────────────────────────────── */
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef  htim5;

/* ── DMA receive buffer ──────────────────────────────────────────────────── */
static uint8_t dma_buf[IBUS_FRAME_LEN];

/* ── Latest decoded state (written by IDLE ISR, read by main loop) ───────── */
static volatile IBUSData_t latest;
static volatile uint32_t   last_good_ts = 0U;

/* ── Private helpers ─────────────────────────────────────────────────────── */

static float _ToFloat(uint16_t raw)
{
    if (raw < IBUS_VAL_MIN) raw = IBUS_VAL_MIN;
    if (raw > IBUS_VAL_MAX) raw = IBUS_VAL_MAX;

    /* IBUS_VAL_* are #define'd with a U suffix (unsigned int). Subtracting
     * them directly forces raw - IBUS_VAL_MID into unsigned arithmetic, so
     * for raw < IBUS_VAL_MID the true negative result wraps to a huge
     * unsigned value instead of going negative — e.g. 1000u - 1500u becomes
     * ~4.29e9, not -500. Cast to int32_t first so the subtraction is signed. */
    int32_t diff = (int32_t)raw - (int32_t)IBUS_VAL_MID;

    float norm;
    if (raw < IBUS_VAL_MID)
        norm = (float)diff / (float)(IBUS_VAL_MID - IBUS_VAL_MIN);
    else
        norm = (float)diff / (float)(IBUS_VAL_MAX - IBUS_VAL_MID);

    if (norm > -DEADBAND && norm < DEADBAND) return 0.0f;
    return norm;
}

static float _SwcToSpeed(uint16_t raw)
{
    if (raw < SWC_THRESH_LO) return SWC_SPEED_LOW;
    if (raw < SWC_THRESH_HI) return SWC_SPEED_MID;
    return SWC_SPEED_HIGH;
}

/* SWB repurposed for Phase 5: MANUAL/AUTO command-source select, not a speed
 * cap anymore. See config.h SWB section and scratchpad.md. */
static uint8_t _SwbToAuto(uint16_t raw)
{
    return (raw >= SWB_THRESH) ? 1U : 0U;
}

static uint8_t _ParseFrame(const uint8_t *buf, uint16_t *ch)
{
    if (buf[0] != IBUS_HEADER_0 || buf[1] != IBUS_HEADER_1) return 0U;

    uint16_t calc = 0xFFFFU;
    for (uint8_t i = 0U; i < 30U; i++)
        calc -= (uint16_t)buf[i];

    uint16_t rx = (uint16_t)buf[30] | ((uint16_t)buf[31] << 8);
    if (calc != rx) return 0U;

    for (uint8_t c = 0U; c < IBUS_NUM_CH; c++)
    {
        uint8_t  idx = 2U + c * 2U;
        uint16_t val = (uint16_t)buf[idx] | ((uint16_t)buf[idx + 1U] << 8);
        if (val < IBUS_VAL_MIN) val = IBUS_VAL_MIN;
        if (val > IBUS_VAL_MAX) val = IBUS_VAL_MAX;
        ch[c] = val;
    }
    return 1U;
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void IBUS_Init(void)
{
    latest.Xn            = 0.0f;
    latest.Yn            = 0.0f;
    latest.fwd_speed_pct = SWC_SPEED_HIGH;
    latest.rev_speed_pct = 1.0f;   /* fixed — see IBUSData_t.rev_speed_pct comment */
    latest.swd_on        = 0U;
    latest.auto_mode     = 0U;     /* default MANUAL until a frame says otherwise */
    latest.rc_ok         = 0U;
    last_good_ts         = 0U;

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, dma_buf, IBUS_FRAME_LEN);
}

void IBUS_Read(IBUSData_t *out)
{
    uint32_t elapsed = TIM5->CNT - last_good_ts;

    __disable_irq();
    if (elapsed >= RC_TIMEOUT_US)
        latest.rc_ok = 0U;
    *out = latest;
    __enable_irq();
}

void IBUS_IdleCallback(void)
{
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    HAL_UART_DMAStop(&huart1);

    uint16_t ch[IBUS_NUM_CH];
    if (_ParseFrame(dma_buf, ch))
    {
        __disable_irq();
        latest.Xn            =  _ToFloat(ch[IBUS_CH_X]);
        latest.Yn            =  _ToFloat(ch[IBUS_CH_Y]);
        latest.fwd_speed_pct =  _SwcToSpeed(ch[IBUS_CH_SWC]);
        latest.rev_speed_pct =  1.0f;   /* fixed — see IBUSData_t.rev_speed_pct comment */
        latest.swd_on        = (ch[IBUS_CH_SWD] > SWD_THRESH) ? 1U : 0U;
        latest.auto_mode     =  _SwbToAuto(ch[IBUS_CH_SWB]);
        latest.rc_ok         =  1U;
        last_good_ts         =  TIM5->CNT;
        __enable_irq();
    }

    HAL_UART_Receive_DMA(&huart1, dma_buf, IBUS_FRAME_LEN);
}
