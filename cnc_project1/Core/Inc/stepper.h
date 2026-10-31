
#ifndef __STEPPER_H
#define __STEPPER_H

#include "main.h"

#define STEPPER_X  0
#define STEPPER_Y  1

#define STEPPER_TICK_HZ 50000UL

/* ????????? */
void Stepper_Init(void);

/* direction: 1??,-1??
   speed: mm/s */
uint8_t Stepper_Start(uint8_t axis,
                      int8_t direction,
                      float speed);

/* ???? */
void Stepper_Stop(uint8_t axis);

/* ?????? */
void Stepper_StopAll(void);

/* TIM2?????? */
void Stepper_TIM_Callback(void);

#endif
