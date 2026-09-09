#include "encoder.h"
#include "config.h"
#include "main.h"
#include <stdint.h>

/* ── External HAL handles (declared in main.c by CubeMX) ─────────────────── */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;

/* ── Last raw hardware counter values, each at its timer's native width ──── */
static uint32_t prev_raw_l = 0U;   /* TIM2 — 32-bit */
static uint16_t prev_raw_r = 0U;   /* TIM3 — 16-bit */

/* ── Accumulated signed counts (telemetry / bench verification) ──────────── */
static int32_t total_count_l = 0;
static int32_t total_count_r = 0;

/* ── TIM5 timestamp of last Encoder_Update call ──────────────────────────── */
static uint32_t prev_ts = 0;

/* ── Timestamps of last movement (for stopped detection) ────────────────── */
static uint32_t last_pulse_ts_l = 0;
static uint32_t last_pulse_ts_r = 0;

/* ── Outputs ─────────────────────────────────────────────────────────────── */
static float rpm_l = 0.0f;
static float rpm_r = 0.0f;


/* ── Helpers ─────────────────────────────────────────────────────────────── */

/* Signed movement between two readings of TIM3's 16-bit counter, correct
 * across counter wrap in either direction.
 *
 * WHY THIS EXISTS: TIM3 is a 16-bit timer, so its counter rolls
 * 65535 -> 0 going forward (or 0 -> 65535 in reverse) roughly every 82 s of
 * continuous rotation at full wheel speed (~4.5 min at the 30 % speed cap).
 * Subtracting the two readings naively gives -65535 instead of +1 at that
 * moment, and the RPM maths turns that into exactly one bogus -32768 RPM
 * sample:  65536 x 1200 / 2400  =  65536 / 2  =  32768.
 *
 * Taking the difference modulo 2^16 and then reading the upper half of the
 * range as negative recovers the true signed movement, with no overflow
 * counter, no interrupt and no flag handling. Written with an explicit
 * comparison rather than a cast because converting an out-of-range value to
 * a signed type is implementation-defined in C — and a subtle arithmetic
 * assumption is exactly what caused the earlier reverse/left-steering bug.
 *
 * Valid as long as the wheel moves fewer than 32767 counts between calls.
 * At 800 counts/s and a 50 ms loop that is ~40 counts — about 800x margin.
 *
 * NOTE: the previous design tried to track wraps with a TIM3 update
 * interrupt instead. Do NOT go back to that. HAL_TIM_Encoder_Start_IT()
 * enables the CC1/CC2 (per-edge) interrupts, not Update, so an ISR that only
 * clears the Update flag never clears the flag that woke it and re-fires
 * forever — that locked the MCU solid as soon as a wheel turned. */
static inline int32_t _Delta16(uint16_t now, uint16_t prev)
{
    uint16_t d = (uint16_t)(now - prev);
    return (d > 32767U) ? ((int32_t)d - 65536) : (int32_t)d;
}


/* ── Public API ──────────────────────────────────────────────────────────── */

void Encoder_Init(void)
{
    /* Plain (non-IT) start for both. TIM3 needs no interrupt — wrap is
     * handled arithmetically in _Delta16(). */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    /* Start microsecond timebase */
    HAL_TIM_Base_Start(&htim5);

    /* Snapshot starting values so the first Encoder_Update() sees a delta
     * measured from here, not from zero. */
    uint32_t now    = TIM5->CNT;
    prev_raw_l      = TIM2->CNT;
    prev_raw_r      = (uint16_t)TIM3->CNT;
    total_count_l   = 0;
    total_count_r   = 0;
    prev_ts         = now;
    last_pulse_ts_l = now;
    last_pulse_ts_r = now;
}

void Encoder_Update(void)
{
    uint32_t now = TIM5->CNT;

    /* Elapsed time in microseconds (handles 32-bit TIM5 rollover) */
    uint32_t dt_us = now - prev_ts;
    if (dt_us == 0U) return;

    uint32_t raw_l = TIM2->CNT;
    uint16_t raw_r = (uint16_t)TIM3->CNT;

    /* TIM2 is 32-bit: the unsigned subtraction wraps correctly and the cast
     * recovers the sign. At full speed it would not wrap for ~62 days, but
     * the idiom costs nothing and stays correct if it ever does. */
    int32_t delta_l = (int32_t)(raw_l - prev_raw_l);

    /* TIM3 is 16-bit and DOES wrap regularly — see _Delta16(). */
    int32_t delta_r = _Delta16(raw_r, prev_raw_r);

    prev_raw_l = raw_l;
    prev_raw_r = raw_r;
    prev_ts    = now;

    /* Running totals — monotonic, so they do not visibly jump at a wrap */
    total_count_l += delta_l;
    total_count_r += delta_r;

    /* Track last-movement time for stopped detection */
    if (delta_l != 0) last_pulse_ts_l = now;
    if (delta_r != 0) last_pulse_ts_r = now;

    /* Stopped detection — no movement for ENC_STOPPED_US */
    if ((now - last_pulse_ts_l) >= ENC_STOPPED_US)
    {
        rpm_l = 0.0f;
    }
    else
    {
        /* rpm = (counts / counts_per_rev) × (60 × 1e6 / dt_us) */
        rpm_l = ((float)delta_l / (float)ENCODER_COUNTS_REV)
                * (60.0f * 1000000.0f / (float)dt_us);
    }

    if ((now - last_pulse_ts_r) >= ENC_STOPPED_US)
    {
        rpm_r = 0.0f;
    }
    else
    {
        rpm_r = ((float)delta_r / (float)ENCODER_COUNTS_REV)
                * (60.0f * 1000000.0f / (float)dt_us);
    }
}

float Encoder_GetRPM_L(void) { return rpm_l; }
float Encoder_GetRPM_R(void) { return rpm_r; }

int32_t Encoder_GetCount_L(void) { return total_count_l; }
int32_t Encoder_GetCount_R(void) { return total_count_r; }
