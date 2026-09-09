/* ============================================================================
 *  actuator.c — Cytron MD10C linear steering actuator driver
 * ============================================================================
 */

#include "actuator.h"
#include "ackermann_config.h"
#include "main.h"
#include <stdint.h>

extern TIM_HandleTypeDef htim1;

#ifndef Actuator_DIR_Pin
#define Actuator_DIR_Pin       GPIO_PIN_5
#define Actuator_DIR_GPIO_Port GPIOA
#endif

#ifndef Actuator_Dir_Pin
#define Actuator_Dir_Pin       Actuator_DIR_Pin
#define Actuator_Dir_GPIO_Port Actuator_DIR_GPIO_Port
#endif

static float last_duty = 0.0f;

void Actuator_Init(void)
{
    HAL_GPIO_WritePin(Actuator_Dir_GPIO_Port, Actuator_Dir_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    last_duty = 0.0f;
}

void Actuator_SetSpeed(float duty_pct, uint8_t extend)
{
    if (duty_pct < 0.0f)   duty_pct = 0.0f;
    if (duty_pct > 100.0f) duty_pct = 100.0f;

    /* Apply mounting polarity */
    uint8_t dir = (ACTUATOR_DIR == 1) ? extend : (uint8_t)(!extend);

    HAL_GPIO_WritePin(Actuator_Dir_GPIO_Port, Actuator_Dir_Pin,
                      dir ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* ARR = 999 (1000 total period steps). When duty=100%, CCR reaches 1000 for 100% continuous ON */
    uint32_t ccr = (uint32_t)((duty_pct * (float)(ACT_PWM_ARR + 1U)) / 100.0f);
    if (ccr > (ACT_PWM_ARR + 1U)) ccr = ACT_PWM_ARR + 1U;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)ccr);

    last_duty = extend ? duty_pct : -duty_pct;
}

void Actuator_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    last_duty = 0.0f;
}

float Actuator_GetDuty(void)
{
    return last_duty;
}
