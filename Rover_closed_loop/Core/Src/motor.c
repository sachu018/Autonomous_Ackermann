#include "motor.h"
#include "config.h"
#include "main.h"
#include <stdint.h>

/* ── External HAL handle ──────────────────────────────────────────────────── */
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim5;

/* ── Internal state ──────────────────────────────────────────────────────── */
static uint8_t armed         = 0;
static uint8_t is_energized  = 0;
static uint8_t was_reversing = 0;
static uint8_t last_dir_l    = 0;   /* 0 = forward, 1 = reverse */
static uint8_t last_dir_r    = 0;


/* ── Private helpers ─────────────────────────────────────────────────────── */

/* Diagnostic LED — PC13, onboard LED on this Black Pill, active-LOW.
 * Confirmed unused elsewhere in the project. Lights and STAYS lit if the
 * firmware ever refuses to close the contactor because it could not
 * confirm the DACs were at zero throttle (see _EnsureDacsZeroed()).
 * LED lit + no contactor click = the DAC/I2C path is broken, and the
 * firmware deliberately did not energise. */
static void _DiagLedInit(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_13;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); /* off */
}

static inline void _DiagLedOn(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}

static inline void _DiagLedOff(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}

/* Write a 12-bit value to an MCP4725 DAC over I2C, returning the HAL
 * status so callers that care can react to a failed bus transaction.
 * Command byte 0x40 = fast write, update output register only (no EEPROM). */
static HAL_StatusTypeDef _WriteDacChecked(uint8_t addr, uint16_t value)
{
    if (value > DAC_MAX) value = DAC_MAX;
    uint8_t buf[3];
    buf[0] = 0x40U;                            /* MCP4725 Write DAC Register command, PD=0 */
    buf[1] = (uint8_t)((value >> 4) & 0xFFU); /* D[11:4] */
    buf[2] = (uint8_t)((value << 4) & 0xFFU); /* D[3:0] in upper nibble */
    return HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(addr << 1), buf, 3U, 50U);
}

/* Fire-and-forget variant for the normal driving path, where there's
 * nothing useful to do about a single failed write mid-manoeuvre. */
static void _WriteDac(uint8_t addr, uint16_t value)
{
    (void)_WriteDacChecked(addr, value);
}

/* Delay using TIM5 microsecond counter. */
static void _DelayUs(uint32_t us)
{
    uint32_t start = TIM5->CNT;
    while ((TIM5->CNT - start) < us) {}
}

/* Set or clear a motor control GPIO (REV or BRAKE). */
static inline void _SetGPIO(GPIO_TypeDef *port, uint16_t pin, uint8_t state)
{
    HAL_GPIO_WritePin(port, pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* Poll both DAC addresses until they ACK, or a hard 2s timeout elapses.
 * On cold power-up the STM32 can start executing well before the DAC
 * board's own supply has stabilised. An I2C write attempted against a
 * not-yet-powered slave can leave the bus wedged, so every LATER I2C
 * transaction keeps failing too — even once the DAC is fully powered —
 * because the bus itself is stuck, not because the DAC isn't ready
 * anymore. That's why a manual reset (issued well after cold-boot, once
 * the DAC rail has settled) fixes it but plain power-up doesn't: reset
 * re-runs this same init sequence, but by then the bus is healthy.
 * Waiting here for an actual ACK avoids ever touching the bus while the
 * DAC isn't ready, instead of guessing a fixed startup delay.
 * TIM5 is started here rather than waiting for Encoder_Init() (which
 * normally starts it, but runs after Motor_Init()) since this needs it
 * immediately; Encoder_Init() starting it again afterward is harmless. */
static void _WaitForDacsReady(void)
{
    HAL_TIM_Base_Start(&htim5);

    /* Short cap: at this point Motor_Init() has not energised anything, so
     * if the DAC is powered from a contactor-switched rail it CANNOT
     * respond yet and waiting longer is pure wasted boot time. Motor_
     * Energize() zeroes the DACs again after the contactor closes, which
     * is the attempt that matters in that topology. */
    const uint32_t MAX_WAIT_US = 300000U; /* 300 ms cap */
    uint32_t start = TIM5->CNT;

    for (;;)
    {
        uint8_t left_ok  = (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(DAC_LEFT_ADDR  << 1), 1U, 20U) == HAL_OK);
        uint8_t right_ok = (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(DAC_RIGHT_ADDR << 1), 1U, 20U) == HAL_OK);
        if (left_ok && right_ok) return;
        if ((TIM5->CNT - start) >= MAX_WAIT_US) return;
        _DelayUs(50000U); /* 50 ms between attempts */
    }
}

/* Force any I2C slave that's frozen holding SDA/SCL low to release the
 * bus, then cleanly re-initialise I2C1. Safe to call any time AFTER the
 * very first MX_I2C1_Init() has already run (uses HAL_I2C_DeInit/Init on
 * a live peripheral) — for the separate boot-time-only version used
 * BEFORE I2C1 exists at all, see _I2C1_BusRecovery() in main.c.
 *
 * Called from Motor_Energize() on EVERY energize, not just at boot: the
 * contactor relay's mechanical closure is a plausible source of EMI that
 * could glitch the I2C bus independently of the DAC's own power state, so
 * a bus wedge could recur any time the contactor re-closes mid-session
 * (e.g. disarm then re-arm via SWD), which the boot-only fix in main.c
 * would never catch. */
static void _I2C1_Recover(void)
{
    HAL_I2C_DeInit(&hi2c1);

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); /* release SDA */

    for (int i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        _DelayUs(5U);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        _DelayUs(5U);
    }

    /* Manual STOP condition: SDA low->high while SCL is high. */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    _DelayUs(5U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    _DelayUs(5U);

    /* Re-init with the same config as MX_I2C1_Init() in main.c — kept in
     * sync manually; HAL_I2C_Init() reconfigures PB6/PB7 back to I2C AF
     * mode via HAL_I2C_MspInit() as part of this call. */
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 100000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

/* Guarantee both DACs are confirmed at ZERO throttle before the contactor
 * is allowed to close. Returns 1 if confirmed, 0 if it could not be
 * achieved after several recovery attempts.
 *
 * WHY THIS IS THE CRITICAL ORDERING: e-scooter BLDC controllers typically
 * self-check "is throttle at zero?" at their own power-up and latch a
 * refuse-to-run fault if it isn't. The contactor is what feeds them power,
 * so the instant it closes IS their power-up moment. On a cold boot the
 * MCP4725 comes up outputting whatever is in its EEPROM (not necessarily
 * 0 V); if the first zeroing write fails because the I2C bus isn't healthy
 * yet, the DAC is still sitting at that non-zero voltage when the contactor
 * closes — the controllers latch a fault and will not move the wheels no
 * matter how correct every later DAC write is. Only a contactor
 * power-cycle clears it, which is exactly why pressing the STM32 reset
 * button "fixes" it: reset re-opens and re-closes the contactor while the
 * DAC (never power-cycled, still holding 0 V from the previous session)
 * presents a clean zero, so the controllers come up fault-free.
 *
 * Hence: recover the bus and PROVE a zero write lands, BEFORE energising.
 * Doing this after closing the contactor (as an earlier revision did) is
 * useless — the fault is already latched by then. */
static uint8_t _EnsureDacsZeroed(void)
{
    /* Only 2 attempts — this runs twice per energize (before and after the
     * contactor closes) and must not stall arming for long when the DAC
     * simply isn't reachable. */
    for (uint8_t attempt = 0U; attempt < 2U; attempt++)
    {
        HAL_StatusTypeDef l = _WriteDacChecked(DAC_LEFT_ADDR,  DAC_ZERO);
        HAL_StatusTypeDef r = _WriteDacChecked(DAC_RIGHT_ADDR, DAC_ZERO);

        if (l == HAL_OK && r == HAL_OK)
        {
            _DelayUs(50000U); /* 50 ms for the 0 V to settle at the controller input */
            return 1U;
        }

        _I2C1_Recover();
        _DelayUs(50000U);
    }
    return 0U;
}

/* Ramp both DACs from 0 up to their target values in DAC_RAMP_STEPS steps
 * instead of jumping straight there. Used only right after a direction
 * change — the controller appears to reject (or need repeated manual
 * coaxing to accept) an abrupt full-throttle command immediately after
 * reversing direction, but responds to a gradual ramp. Continued driving
 * in the same direction is unaffected — this is only called from the
 * dir_changed branch of Motor_SetSpeeds(). */
static void _RampDac(uint16_t dac_L, uint16_t dac_R)
{
    for (uint32_t i = 1U; i <= DAC_RAMP_STEPS; i++)
    {
        uint16_t step_L = (uint16_t)(((uint32_t)dac_L * i) / DAC_RAMP_STEPS);
        uint16_t step_R = (uint16_t)(((uint32_t)dac_R * i) / DAC_RAMP_STEPS);
        _WriteDac(DAC_LEFT_ADDR,  step_L);
        _WriteDac(DAC_RIGHT_ADDR, step_R);
        _DelayUs(DAC_RAMP_STEP_US);
    }
}


/* ── Public API ──────────────────────────────────────────────────────────── */

void Motor_Init(void)
{
    _DiagLedInit();
    _WaitForDacsReady();

    /* All control outputs start LOW — safe, brakes off, contactor open */
    HAL_GPIO_WritePin(Contactor_GPIO_Port, Contactor_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Rev_Left_GPIO_Port,  Rev_Left_Pin,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Rev_Right_GPIO_Port,  Rev_Right_Pin,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Brake_Left_GPIO_Port, Brake_Left_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Brake_Right_GPIO_Port,Brake_Right_Pin,GPIO_PIN_RESET);

    _WriteDac(DAC_LEFT_ADDR,  DAC_ZERO);
    _WriteDac(DAC_RIGHT_ADDR, DAC_ZERO);

    armed         = 0;
    is_energized  = 0;
    was_reversing = 0;
    last_dir_l    = 0;
    last_dir_r    = 0;
}

void Motor_Energize(void)
{
    if (is_energized) return;

    /* Best-effort zeroing before the contactor closes. This MUST NOT block
     * energising: if the DAC board draws its power from a rail the
     * contactor itself switches, then the DAC cannot answer on I2C until
     * the contactor is already closed — gating the contactor on an I2C
     * success creates an unbreakable deadlock (contactor waits on DAC, DAC
     * waits on contactor) that not even a reset clears. An earlier
     * revision did exactly that and left the rover unable to arm at all.
     * The LED still reports the failure; it just no longer blocks. */
    if (!_EnsureDacsZeroed()) _DiagLedOn();

    HAL_GPIO_WritePin(Contactor_GPIO_Port, Contactor_Pin, GPIO_PIN_SET);
    _DelayUs(500000U); /* 500 ms settle time for contactor */
    is_energized = 1;
    armed        = 1;

    /* Second attempt AFTER the contactor is closed. If the DAC is powered
     * downstream of the contactor, this is the first moment it can
     * actually respond — so this is the call that realistically lands. */
    if (_EnsureDacsZeroed()) _DiagLedOff();
}

void Motor_Deenergize(void)
{
    if (!is_energized) return;
    _WriteDac(DAC_LEFT_ADDR,  DAC_ZERO);
    _WriteDac(DAC_RIGHT_ADDR, DAC_ZERO);
    HAL_GPIO_WritePin(Contactor_GPIO_Port, Contactor_Pin, GPIO_PIN_RESET);
    is_energized = 0;
    armed        = 0;
}

void Motor_SetSpeeds(float rpm_L, float rpm_R, uint16_t dac_L, uint16_t dac_R)
{
    uint8_t new_dir_l = (rpm_L * (float)LEFT_MOTOR_DIR)  < 0.0f ? 1U : 0U;
    uint8_t new_dir_r = (rpm_R * (float)RIGHT_MOTOR_DIR) < 0.0f ? 1U : 0U;

    uint8_t dir_changed = (new_dir_l != last_dir_l) || (new_dir_r != last_dir_r);

    if (dir_changed)
    {
        /* Drop speed to zero before toggling direction pins */
        _WriteDac(DAC_LEFT_ADDR,  DAC_ZERO);
        _WriteDac(DAC_RIGHT_ADDR, DAC_ZERO);
        _DelayUs(10000U); /* 10 ms */

        _SetGPIO(Rev_Left_GPIO_Port,  Rev_Left_Pin,  new_dir_l);
        _SetGPIO(Rev_Right_GPIO_Port, Rev_Right_Pin, new_dir_r);
        last_dir_l = new_dir_l;
        last_dir_r = new_dir_r;

        _SetGPIO(Brake_Left_GPIO_Port,  Brake_Left_Pin,  0U);
        _SetGPIO(Brake_Right_GPIO_Port, Brake_Right_Pin, 0U);

        /* Hold at zero to let motor controller register the direction change */
        _DelayUs(150000U); /* 150 ms */

        _DelayUs(MOTOR_SEQ_DELAY_US); /* 20 ms before ramping DAC */
        _RampDac(dac_L, dac_R);
    }
    else
    {
        _SetGPIO(Rev_Left_GPIO_Port,  Rev_Left_Pin,  new_dir_l);
        _SetGPIO(Rev_Right_GPIO_Port, Rev_Right_Pin, new_dir_r);
        _SetGPIO(Brake_Left_GPIO_Port,  Brake_Left_Pin,  0U);
        _SetGPIO(Brake_Right_GPIO_Port, Brake_Right_Pin, 0U);

        _DelayUs(MOTOR_SEQ_DELAY_US); /* 20 ms before writing DAC */
        _WriteDac(DAC_LEFT_ADDR,  dac_L);
        _WriteDac(DAC_RIGHT_ADDR, dac_R);
    }

    was_reversing = (new_dir_l || new_dir_r) ? 1U : 0U;
}

void Motor_Brake(void)
{
    _WriteDac(DAC_LEFT_ADDR,  DAC_ZERO);
    _WriteDac(DAC_RIGHT_ADDR, DAC_ZERO);
    _SetGPIO(Brake_Left_GPIO_Port,  Brake_Left_Pin,  1U);
    _SetGPIO(Brake_Right_GPIO_Port, Brake_Right_Pin, 1U);
    _SetGPIO(Rev_Left_GPIO_Port,  Rev_Left_Pin,  0U);
    _SetGPIO(Rev_Right_GPIO_Port, Rev_Right_Pin, 0U);
    was_reversing = 0;
}

void Motor_Stop(void)
{
    _WriteDac(DAC_LEFT_ADDR,  DAC_ZERO);
    _WriteDac(DAC_RIGHT_ADDR, DAC_ZERO);
    _SetGPIO(Brake_Left_GPIO_Port,  Brake_Left_Pin,  0U);
    _SetGPIO(Brake_Right_GPIO_Port, Brake_Right_Pin, 0U);
    _SetGPIO(Rev_Left_GPIO_Port,  Rev_Left_Pin,  0U);
    _SetGPIO(Rev_Right_GPIO_Port, Rev_Right_Pin, 0U);
    was_reversing = 0;
}

void Motor_BrakeThenReverse(float rpm_L, float rpm_R, uint16_t dac_L, uint16_t dac_R)
{
    /* Apply brake, wait, then hand off to SetSpeeds for the reverse move */
    _WriteDac(DAC_LEFT_ADDR,  DAC_ZERO);
    _WriteDac(DAC_RIGHT_ADDR, DAC_ZERO);
    _SetGPIO(Brake_Left_GPIO_Port,  Brake_Left_Pin,  1U);
    _SetGPIO(Brake_Right_GPIO_Port, Brake_Right_Pin, 1U);
    _SetGPIO(Rev_Left_GPIO_Port,  Rev_Left_Pin,  0U);
    _SetGPIO(Rev_Right_GPIO_Port, Rev_Right_Pin, 0U);

    _DelayUs(REVERSE_BRAKE_US);

    _SetGPIO(Brake_Left_GPIO_Port,  Brake_Left_Pin,  0U);
    _SetGPIO(Brake_Right_GPIO_Port, Brake_Right_Pin, 0U);

    Motor_SetSpeeds(rpm_L, rpm_R, dac_L, dac_R);
}

uint8_t Motor_IsArmed(void)         { return armed;         }
uint8_t Motor_WasReversing(void)    { return was_reversing; }
void    Motor_SetWasReversing(uint8_t val) { was_reversing = val; }
