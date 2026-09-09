/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* NOTE: these must stay inside this USER CODE block, not above it — CubeMX
 * regenerates everything outside USER CODE markers from scratch on every
 * "Generate Code", and a prior edit that added these directly above (outside
 * any marker) was silently wiped by today's regeneration for the RPi UART
 * link. Re-added here so they survive the next regeneration too. See
 * scratchpad.md "Missing Includes Wiped by CubeMX Regeneration". */
#include "config.h"
#include "ackermann_config.h"
#include "ibus.h"
#include "rpi_link.h"
#include "motor.h"
#include "encoder.h"
#include "ads1115.h"
#include "actuator.h"
#include "steer_pid.h"
#include "ackermann.h"
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Contactor edge-detect state — local to main loop */
static uint8_t      prev_swd_on       = 0U;
static uint32_t     last_contactor_ts = 0U;

/* Closed-loop steering state */
static SteerPID_t   steer_pid;
static SteerAngles_t steer;
static uint32_t     prev_loop_ts      = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ── Debug output over USART1 TX (PA9) → ESP32 → USB serial monitor ───────
 *
 * Writes USART1's data register DIRECTLY rather than going through
 * HAL_UART_Transmit(). This is deliberate and important:
 *
 *   HAL_UART_Transmit() takes __HAL_LOCK(huart1) and holds it for the whole
 *   blocking send. If the iBUS IDLE interrupt fires during that window, the
 *   HAL_UART_Receive_DMA() call inside IBUS_IdleCallback() returns HAL_BUSY,
 *   the RX DMA is never restarted, and RC frames get dropped. With frames
 *   arriving every ~7 ms and a send taking several ms, that would collide
 *   constantly — unacceptable on a machine with powered motors.
 *
 * TX and RX are fully independent paths inside the USART, so writing DR for
 * transmit cannot disturb the iBUS RX DMA, and bypassing HAL means the lock
 * is never touched.
 *
 * The TXE busy-wait is hardware-bounded (~87 us/char at 115200) because the
 * transmitter always drains, whether or not anything is listening on PA9 —
 * so unlike the ITM/SWV approach this can never stall the 20 Hz control
 * loop, and it works with nothing connected at all. */
static void _DbgPutc(char c)
{
    while ((USART1->SR & USART_SR_TXE) == 0U) { }
    USART1->DR = (uint8_t)c;
}

static void _DbgPuts(const char *s)
{
    while (*s != '\0') _DbgPutc(*s++);
}

/* Format an RPM as e.g. "+12.50". Deliberately avoids %f — newlib-nano
 * omits float formatting unless -u _printf_float is added to the linker
 * flags, which silently prints nothing or garbage otherwise. */
static void _FmtRpm(char *out, int n, float rpm)
{
    int32_t v = (int32_t)(rpm * 100.0f);
    char sign = (v < 0) ? '-' : '+';
    if (v < 0) v = -v;
    snprintf(out, n, "%c%ld.%02ld", sign, (long)(v / 100), (long)(v % 100));
}

/* I2C1 bus-clear recovery, run on PB6 (SCL) / PB7 (SDA) as plain open-drain
 * GPIO, BEFORE MX_I2C1_Init() ever reconfigures those pins to I2C
 * alternate-function mode.
 *
 * Why this exists: on a cold boot, the DAC board's own supply rail can
 * still be ramping up when this MCU starts executing. If an I2C
 * transaction is attempted against the DAC while it's only partially
 * powered, the DAC can end up frozen mid-transaction, actively holding
 * SDA low. Neither a hardware NRST reset nor a software NVIC_SystemReset()
 * fixes this — a reset only clears THIS chip's own I2C peripheral state,
 * it cannot force an external slave that's actively driving a line low to
 * let go of it. (Confirmed against STMicroelectronics community reports of
 * the identical symptom on STM32F4 I2C.) The only real fix, per the I2C-bus
 * spec's "bus clear" procedure: manually clock SCL up to 9 times while SDA
 * is released, so the stuck slave's internal shift register finishes
 * clocking out whatever byte it was frozen on and releases the line, then
 * generate a manual STOP condition to leave the bus cleanly idle.
 *
 * Runs unconditionally on every boot (not just cold power-on) — it's fast
 * (a few hundred microseconds) and harmless if the bus is already idle,
 * since without a preceding START condition a healthy slave simply ignores
 * the extra clock edges. */
static void _I2C1_BusRecovery(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* Release SDA (external 4.7k pull-up brings it high) — only SCL is
     * actively driven during the clock-out phase. */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

    for (int i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        for (volatile int d = 0; d < 4000; d++) { __NOP(); }
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        for (volatile int d = 0; d < 4000; d++) { __NOP(); }
    }

    /* Manual STOP condition: SDA transitions low->high while SCL is high. */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    for (volatile int d = 0; d < 4000; d++) { __NOP(); }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    for (volatile int d = 0; d < 4000; d++) { __NOP(); }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* Bus-recovery alone (see _I2C1_BusRecovery() above) did not resolve the
   * cold-boot no-movement issue on its own. Bringing back a delay on a
   * genuine cold boot ONLY (RCC_FLAG_PORRST/BORRST — not a later manual
   * reset, RCC_FLAG_PINRST) so the DAC board's supply rail is fully
   * settled before recovery even runs, rather than possibly bit-banging a
   * still-powering-up device. No self-reset this time — that part was
   * tested and confirmed to add nothing (a reset can't affect an external
   * slave's state either way), so it's not worth the extra boot time. */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) || __HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))
  {
      HAL_Delay(3000);
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();

  /* Must run before MX_I2C1_Init() (further down) ever reconfigures
   * PB6/PB7 into I2C alternate-function mode. */
  _I2C1_BusRecovery();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  Motor_Init();
  Encoder_Init();
  IBUS_Init();
  RPiLink_Init();
  ADS1115_Init();
  Actuator_Init();
  SteerPID_Init(&steer_pid, KP_STEER, KI_STEER, KD_STEER);
  Motor_Energize();               /* auto-arm on boot, contactor click = system ready */
  last_contactor_ts = TIM5->CNT; /* debounce timer starts from this moment */
  prev_loop_ts      = TIM5->CNT;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    uint32_t t0 = TIM5->CNT;

    /* ── 1. Read RC receiver ────────────────────────────────────────────── */
    IBUSData_t rc;
    IBUS_Read(&rc);

    /* ── 1b. Read RPi autonomous link (Phase 5) ─────────────────────────── */
    RPiCmd_t rpi_cmd;
    RPiLink_Read(&rpi_cmd);

    /* ── 2. Contactor edge-detect + 500 ms debounce ─────────────────────── */
    uint8_t rising  =  rc.swd_on && !prev_swd_on;
    uint8_t falling = !rc.swd_on &&  prev_swd_on;

    if (rising || falling)
    {
        uint32_t elapsed = TIM5->CNT - last_contactor_ts;
        if (elapsed >= CONTACTOR_DEBOUNCE_US)
        {
            if (rising)  Motor_Energize();
            else         { Motor_Stop(); Motor_Deenergize(); }
            last_contactor_ts = TIM5->CNT;
        }
    }
    prev_swd_on = rc.swd_on;

    /* ── 3. Time elapsed for PID (µs from TIM5) ─────────────────────────── */
    uint32_t now_ts = TIM5->CNT;
    float dt_s = (float)(now_ts - prev_loop_ts) / 1000000.0f;
    prev_loop_ts = now_ts;

    /* ── 4. Read steering potentiometer feedback via ADS1115 ────────────── */
    ADS1115_Read(&steer);

    /* ── 5. Ackermann motion pipeline & electronic differential ──────────── */
    AckResult_t ack;

    if (Motor_IsArmed() && rc.rc_ok)
    {
        if (rc.auto_mode)
        {
            /* SWB = AUTO: command source is the RPi UART link, gated on its
             * own health independent of the RC iBUS link (rc.rc_ok only
             * tells us SWB's own position is fresh, not whether the RPi is
             * alive). Link loss while AUTO is selected -> brake, same
             * failsafe class as RC loss — see architecture.md §3.5. */
            if (rpi_cmd.link_ok)
            {
                Ackermann_RunAuto(rpi_cmd.steer_target_deg, rpi_cmd.speed_target_ms,
                                  steer.delta, &ack);
            }
            else
            {
                ack.state            = ACK_STATE_BRAKE;
                ack.target_steer_deg = 0.0f;
                ack.V_base            = 0.0f;
                ack.rpm_L             = 0.0f;
                ack.rpm_R             = 0.0f;
                ack.dac_L             = 0U;
                ack.dac_R             = 0U;
            }
        }
        else
        {
            Ackermann_Run(rc.Xn, rc.Yn,
                          rc.fwd_speed_pct, rc.rev_speed_pct,
                          steer.delta, rc.rc_ok, &ack);
        }
    }
    else
    {
        ack.state            = Motor_IsArmed() ? ACK_STATE_NO_SIGNAL : ACK_STATE_DISARMED;
        ack.target_steer_deg = 0.0f;
        ack.rpm_L            = 0.0f;
        ack.rpm_R            = 0.0f;
        ack.dac_L            = 0U;
        ack.dac_R            = 0U;
    }

    /* ── 6. Steering actuator closed-loop control ──────────────────────────
     * Drives the actuator at full 100% speed (7 mm/s) in the direction of
     * the target angle until within STEER_DEADBAND_DEG (±0.5°).
     * Runs on RC validity + sensor health (safe to test with drive unpowered). */
    if (rc.rc_ok && steer.valid)
    {
        float err = ack.target_steer_deg - steer.delta;
        if (fabsf(err) <= STEER_DEADBAND_DEG)
        {
            Actuator_Stop();
        }
        else
        {
            /* Positive error (target > actual): drive Left/Extend at 100%
             * Negative error (target < actual): drive Right/Retract at 100% */
            Actuator_SetSpeed(100.0f, (uint8_t)(err > 0.0f));
        }
    }
    else
    {
        Actuator_Stop();
    }

    /* ── 7. Drive rear motors ────────────────────────────────────────────── */
    uint8_t is_rev = (ack.state == ACK_STATE_REV      ||
                      ack.state == ACK_STATE_REV_LEFT  ||
                      ack.state == ACK_STATE_REV_RIGHT);

    switch (ack.state)
    {
        case ACK_STATE_BRAKE:
        case ACK_STATE_NO_SIGNAL:
            Motor_Brake();
            Motor_SetWasReversing(0U);
            break;

        case ACK_STATE_DISARMED:
            Motor_Stop();
            Motor_SetWasReversing(0U);
            break;

        default:
            if (is_rev && !Motor_WasReversing())
                Motor_BrakeThenReverse(ack.rpm_L, ack.rpm_R,
                                       ack.dac_L, ack.dac_R);
            else
                Motor_SetSpeeds(ack.rpm_L, ack.rpm_R,
                                ack.dac_L, ack.dac_R);
            break;
    }

    /* ── 8. Update encoder RPM ───────────────────────────────────────────── */
    Encoder_Update();

    /* ── 8b. Send feedback frame to the RPi over USART2, every tick (20 Hz) ─
     * Phase 5: lets the RPi's guidance loop see measured steering angle and
     * wheel RPM regardless of which mode (MANUAL/AUTO) is currently active,
     * so it can track state and be ready the instant SWB flips to AUTO. */
    {
        uint8_t status = 0U;
        if (Motor_IsArmed())    status |= RPI_STATUS_ARMED;
        if (!steer.valid)       status |= RPI_STATUS_STEER_FAULT;
        if (rc.auto_mode)       status |= RPI_STATUS_AUTO_ACTIVE;
        if (rc.rc_ok)           status |= RPI_STATUS_RC_OK;

        RPiLink_SendFeedback(steer.angle_L, steer.angle_R,
                             Encoder_GetRPM_L(), Encoder_GetRPM_R(), status);
    }

    /* ── 9. Debug telemetry over USART1 TX → ESP32, rate-limited to ~4 Hz ─── */
    static uint8_t dbg_div = 0U;
    if (++dbg_div >= 5U)
    {
        dbg_div = 0U;
        char cl[12], cr[12], el[12], er[12], ts[12], as[12], line[160];
        _FmtRpm(cl, sizeof cl, ack.rpm_L);
        _FmtRpm(cr, sizeof cr, ack.rpm_R);
        _FmtRpm(el, sizeof el, Encoder_GetRPM_L());
        _FmtRpm(er, sizeof er, Encoder_GetRPM_R());
        _FmtRpm(ts, sizeof ts, ack.target_steer_deg);
        _FmtRpm(as, sizeof as, steer.delta);
        snprintf(line, sizeof line,
                 "STR tgt%s act%s dty%+4d %s | CMD L%s R%s | ENC L%s R%s\r\n",
                 ts, as, (int)Actuator_GetDuty(),
                 steer.valid ? "OK " : "ERR",
                 cl, cr, el, er);
        _DbgPuts(line);
    }

    /* ── 10. Rate-limit to LOOP_HZ (busy-wait on TIM5) ──────────────────── */
    while ((TIM5->CNT - t0) < LOOP_PERIOD_US) {}

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
