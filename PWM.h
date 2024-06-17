/*
 * PWM.h
 *
 *  Created on: Mar 6, 2023
 *      Author: Matthew Atagi
 *This include file is for the PWM driver on the STM32F410R8T6
 */

#ifndef SRC_TIMERS_H_
#define SRC_TIMERS_H_

#include <stm32f410rx.h>
#include <stm32f4xx.h>
#include <stdint.h>

#define vu16 volatile uint16_t
#define vu32 volatile uint32_t

#define v16 volatile int16_t
#define v32 volatile int32_t


typedef enum  {

	Counter = 0,
	PWM,
	One_Pulse_Mode

} TIM_MODE;

void INIT_TIM_INIT(TIM_TypeDef* TIMx, vu16 Auto_Reload_Preload_Enable, vu16  Alignment, vu16 One_Pulse_Mode, vu16 Count_Direction,  vu16 Update_Request_Source, vu16 Prescaler, vu16 Clock_Division, vu16 Update_Disable);

void INIT_TIM_PWM(TIM_TypeDef* TIMx, vu16 Auto_Reload_Register, vu16 Prescalar_Register);

void Start_PWM(TIM_TypeDef* TIMx,unsigned int TIM_CH, float Duty_Cycle_Percent, GPIO_TypeDef* GPIOx, unsigned int PIN, vu32 AF);

void Start_TIM(TIM_TypeDef* TIMx, TIM_MODE Mode);

void Stop_TIM(TIM_TypeDef* TIMx);

void Set_Duty_Cycle(TIM_TypeDef* TIMx,unsigned int TIM_CH, float Duty_Cycle_Percent);

#endif /* SRC_TIMERS_H_ */
