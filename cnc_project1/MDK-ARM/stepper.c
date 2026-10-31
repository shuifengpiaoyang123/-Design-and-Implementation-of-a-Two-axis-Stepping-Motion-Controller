
#include "stepper.h"
#include "tim.h"
#include <stdint.h>

#define PULSE_PER_REV 1600.0f
#define SCREW_LEAD_MM 5.0f

typedef struct
{
    GPIO_TypeDef *step_port;
    uint16_t step_pin;

    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;

    volatile uint32_t phase;
    volatile uint32_t increment;

    volatile uint32_t pending_increment;
    volatile uint16_t start_wait;

    volatile uint8_t pulse_active;
    volatile int8_t direction;

    volatile int32_t position;

} StepperAxis;

static StepperAxis motor[2];

/* GPIO??? */
void Stepper_Init(void)
{
    motor[STEPPER_X].step_port = X_STEP_GPIO_Port;
    motor[STEPPER_X].step_pin  = X_STEP_Pin;
    motor[STEPPER_X].dir_port  = X_DIR_GPIO_Port;
    motor[STEPPER_X].dir_pin   = X_DIR_Pin;

    motor[STEPPER_Y].step_port = Y_STEP_GPIO_Port;
    motor[STEPPER_Y].step_pin  = Y_STEP_Pin;
    motor[STEPPER_Y].dir_port  = Y_DIR_GPIO_Port;
    motor[STEPPER_Y].dir_pin   = Y_DIR_Pin;

    for (int i = 0; i < 2; i++)
    {
        motor[i].phase = 0;
        motor[i].increment = 0;
        motor[i].pending_increment = 0;
        motor[i].start_wait = 0;
        motor[i].pulse_active = 0;
        motor[i].direction = 1;
        motor[i].position = 0;

        /* STEP??:?????? */
        HAL_GPIO_WritePin(
            motor[i].step_port,
            motor[i].step_pin,
            GPIO_PIN_SET);
    }
}

/* ???? */
uint8_t Stepper_Start(uint8_t id,
                      int8_t direction,
                      float speed)
{
    uint32_t frequency;
    uint64_t increment;

    if (id > STEPPER_Y)
        return 0;

    if (direction != 1 && direction != -1)
        return 0;

    if (speed < 0.1f || speed > 50.0f)
        return 0;

    /*
       1600??/?
       ????5mm
    */
    frequency = (uint32_t)(
        speed * PULSE_PER_REV / SCREW_LEAD_MM);

    increment =
        ((uint64_t)frequency * 4294967296ULL)
        / STEPPER_TICK_HZ;

    if (increment == 0 ||
        increment > 0x80000000ULL)
        return 0;

    __disable_irq();

    if (motor[id].increment != 0 ||
        motor[id].start_wait != 0)
    {
        __enable_irq();
        return 0;
    }

    motor[id].direction = direction;

    /*
       ????
       ??????????,
       ?????????????
    */
    HAL_GPIO_WritePin(
        motor[id].dir_port,
        motor[id].dir_pin,
        direction > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);

    motor[id].phase = 0;
    motor[id].pending_increment =
        (uint32_t)increment;

    /* ??????:1ms */
    motor[id].start_wait = 50;

    __enable_irq();

    return 1;
}

/* ?????? */
void Stepper_Stop(uint8_t id)
{
    if (id > STEPPER_Y)
        return;

    __disable_irq();

    motor[id].increment = 0;
    motor[id].pending_increment = 0;
    motor[id].start_wait = 0;
    motor[id].phase = 0;

    HAL_GPIO_WritePin(
        motor[id].step_port,
        motor[id].step_pin,
        GPIO_PIN_SET);

    motor[id].pulse_active = 0;

    __enable_irq();
}

/* ?????? */
void Stepper_StopAll(void)
{
    Stepper_Stop(STEPPER_X);
    Stepper_Stop(STEPPER_Y);
}

/* ?????? */
static void Stepper_Axis_Tick(StepperAxis *m)
{
    uint32_t old_phase;

    /* ??????? */
    if (m->pulse_active)
    {
        HAL_GPIO_WritePin(
            m->step_port,
            m->step_pin,
            GPIO_PIN_SET);

        m->pulse_active = 0;
    }

    /* ???????? */
    if (m->start_wait > 0)
    {
        m->start_wait--;

        if (m->start_wait == 0)
        {
            m->increment = m->pending_increment;
        }

        return;
    }

    if (m->increment == 0)
        return;

    old_phase = m->phase;

    /* 32?????? */
    m->phase += m->increment;

    /* ???????? */
    if (m->phase < old_phase)
    {
        HAL_GPIO_WritePin(
            m->step_port,
            m->step_pin,
            GPIO_PIN_RESET);

        m->pulse_active = 1;

        if (m->direction > 0)
            m->position++;
        else
            m->position--;
    }
}

/* TIM2?? */
void Stepper_TIM_Callback(void)
{
    Stepper_Axis_Tick(&motor[STEPPER_X]);
    Stepper_Axis_Tick(&motor[STEPPER_Y]);
}
