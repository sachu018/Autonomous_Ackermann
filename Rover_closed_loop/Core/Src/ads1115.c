/* ============================================================================
 *  ads1115.c — ADS1115 external ADC driver (steering potentiometer feedback)
 * ============================================================================
 */

#include "ads1115.h"
#include "ackermann_config.h"
#include "config.h"
#include "main.h"
#include <stdint.h>

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim5;

/* ── ADS1115 registers ──────────────────────────────────────────────────── */
#define ADS_REG_CONVERSION      0x00U
#define ADS_REG_CONFIG          0x01U

/* Config words:
 * OS=1, PGA=001 (+/-4.096V for 3.3V pots), MODE=1 (single-shot), DR=111 (860 SPS) */
#define ADS_CFG_AIN0            0xC3E3U  /* Left pot  (AIN0 vs GND) */
#define ADS_CFG_AIN1            0xD3E3U  /* Right pot (AIN1 vs GND) */

#define ADS_I2C_TIMEOUT_MS      20U

static uint8_t healthy         = 0U;
static uint8_t fault_runs      = 0U;
static SteerAngles_t last_good = {0.0f, 0.0f, 0.0f, 0U};

/* ── Helpers ────────────────────────────────────────────────────────────── */

static void _DelayUs(uint32_t us)
{
    uint32_t start = TIM5->CNT;
    while ((TIM5->CNT - start) < us) { }
}

/* Start conversion on one channel, wait for it, return 16-bit signed result */
static uint8_t _ReadChannel(uint16_t cfg, int16_t *out)
{
    uint8_t buf[3];

    /* Write CONFIG (big-endian) to begin single-shot conversion */
    buf[0] = ADS_REG_CONFIG;
    buf[1] = (uint8_t)(cfg >> 8);
    buf[2] = (uint8_t)(cfg & 0xFFU);
    if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(ADS1115_I2C_ADDR << 1),
                                buf, 3U, ADS_I2C_TIMEOUT_MS) != HAL_OK)
        return 0U;

    /* 860 SPS conversion takes ~1.16 ms; 2 ms wait ensures complete sample */
    _DelayUs(2000U);

    /* Point to conversion register */
    buf[0] = ADS_REG_CONVERSION;
    if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(ADS1115_I2C_ADDR << 1),
                                buf, 1U, ADS_I2C_TIMEOUT_MS) != HAL_OK)
        return 0U;

    /* Read 16-bit conversion result */
    if (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(ADS1115_I2C_ADDR << 1),
                               buf, 2U, ADS_I2C_TIMEOUT_MS) != HAL_OK)
        return 0U;

    uint16_t u = (uint16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
    *out = (u > 32767U) ? (int16_t)((int32_t)u - 65536) : (int16_t)u;

    return 1U;
}

/* Map raw ADC reading to angle in degrees (-45° to +45°) using a 3-point
 * (piecewise-linear) calibration: min_raw->center_raw maps to -MAX..0,
 * center_raw->max_raw maps to 0..+MAX. This guarantees center reads exactly
 * 0° and each lock reaches exactly +-MAX_STEER_ANGLE_DEG, independent of
 * how far either lock's raw value is from center — unlike a single
 * min->max line with a flat degree trim, which compresses one side's
 * range and saturates the other early (see ackermann_config.h). */
static float _MapToAngle(int16_t raw, int32_t min_raw, int32_t center_raw, int32_t max_raw)
{
    float angle;

    if (raw >= center_raw)
    {
        if (max_raw == center_raw) return 0.0f;
        float norm = (float)((int32_t)raw - center_raw) / (float)(max_raw - center_raw);
        angle = norm * MAX_STEER_ANGLE_DEG;
    }
    else
    {
        if (center_raw == min_raw) return 0.0f;
        float norm = (float)(center_raw - (int32_t)raw) / (float)(center_raw - min_raw);
        angle = -norm * MAX_STEER_ANGLE_DEG;
    }

    if (angle >  MAX_STEER_ANGLE_DEG) angle =  MAX_STEER_ANGLE_DEG;
    if (angle < -MAX_STEER_ANGLE_DEG) angle = -MAX_STEER_ANGLE_DEG;
    return angle;
}

static uint8_t _RawIsSane(int16_t raw)
{
    return (raw >= ADC_SANITY_MIN && raw <= ADC_SANITY_MAX) ? 1U : 0U;
}

/* ── Public API ─────────────────────────────────────────────────────────── */

uint8_t ADS1115_Init(void)
{
    healthy = (HAL_I2C_IsDeviceReady(&hi2c1,
                                     (uint16_t)(ADS1115_I2C_ADDR << 1),
                                     3U, ADS_I2C_TIMEOUT_MS) == HAL_OK)
              ? 1U : 0U;
    fault_runs = healthy ? 0U : STEER_FAULT_THRESHOLD;
    last_good.angle_L = 0.0f;
    last_good.angle_R = 0.0f;
    last_good.delta   = 0.0f;
    last_good.valid   = 0U;
    return healthy;
}

uint8_t ADS1115_ReadRaw(int16_t *raw_L, int16_t *raw_R)
{
    int16_t l = 0, r = 0;
    if (!_ReadChannel(ADS_CFG_AIN0, &l)) return 0U;
    if (!_ReadChannel(ADS_CFG_AIN1, &r)) return 0U;
    *raw_L = l;
    *raw_R = r;
    return 1U;
}

void ADS1115_Read(SteerAngles_t *out)
{
    int16_t raw_L = 0, raw_R = 0;

    if (!ADS1115_ReadRaw(&raw_L, &raw_R) ||
        !_RawIsSane(raw_L) || !_RawIsSane(raw_R))
    {
        /* Increment consecutive fault counter */
        if (fault_runs < STEER_FAULT_THRESHOLD) fault_runs++;

        if (fault_runs >= STEER_FAULT_THRESHOLD)
        {
            /* Persistent failure declared */
            healthy    = 0U;
            out->angle_L = 0.0f;
            out->angle_R = 0.0f;
            out->delta   = 0.0f;
            out->valid   = 0U;
            last_good.valid = 0U;
        }
        else
        {
            /* Transient glitch: retain last good angle and keep valid = 1 */
            *out = last_good;
        }
        return;
    }

    /* Healthy reading received */
    fault_runs = 0U;
    healthy    = 1U;

    out->angle_L = _MapToAngle(raw_L, ADC_L_MIN_RAW, ADC_L_CENTER_RAW, ADC_L_MAX_RAW);
    out->angle_R = _MapToAngle(raw_R, ADC_R_MIN_RAW, ADC_R_CENTER_RAW, ADC_R_MAX_RAW);
    out->delta   = (out->angle_L + out->angle_R) * 0.5f;
    out->valid   = 1U;

    last_good = *out;
}

uint8_t ADS1115_IsHealthy(void) { return healthy; }
