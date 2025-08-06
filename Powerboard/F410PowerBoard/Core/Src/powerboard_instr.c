/*
 * powerboard_inst.c
 *This file contains functions for actions taken for each pin of Powerboard based on instructions
 *  Created on: Mar 31, 2024
 *      Author: calum
 */
#include "powerboard_instr.h"
#include "pin_lib.h"
#include <string.h>
#include "main.h"

//GPIO output on/off--------------------------------------------------------
void pulse_enables_clock(){//SET AND RESET PB11 CLOCK FOR BATT, 5V, 12V, 16V ENABLES
	HAL_GPIO_WritePin(GPIOB, ENABLES_CLK, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, ENABLES_CLK, GPIO_PIN_RESET);
}

void e_parallel_off(){
	HAL_GPIO_WritePin(GPIOB, P_PARALLEL_E, GPIO_PIN_RESET);
}
void e_parallel_on(){
	HAL_GPIO_WritePin(GPIOB, P_PARALLEL_E, GPIO_PIN_SET);
}
void micropower_off(){
	HAL_GPIO_WritePin(GPIOC, P_MICROPOWER, GPIO_PIN_RESET);
}
void micropower_on(){
	HAL_GPIO_WritePin(GPIOC, P_MICROPOWER, GPIO_PIN_SET);
}
void e_vbatt_off(){
	HAL_GPIO_WritePin(GPIOC, P_VBATT_E, GPIO_PIN_RESET);
	pulse_enables_clock();
}
void e_vbatt_on(){
	HAL_GPIO_WritePin(GPIOC, P_VBATT_E, GPIO_PIN_SET);
	pulse_enables_clock();
}
void e_5v_off(){
	HAL_GPIO_WritePin(GPIOC, P_5V_E, GPIO_PIN_RESET);
	pulse_enables_clock();
}
void e_5v_on(){
	HAL_GPIO_WritePin(GPIOC, P_5V_E, GPIO_PIN_SET);
	pulse_enables_clock();
}
void e_12v_off(){
	HAL_GPIO_WritePin(GPIOC, P_12V_E, GPIO_PIN_RESET);
	pulse_enables_clock();
}
void e_12v_on(){
	HAL_GPIO_WritePin(GPIOC, P_12V_E, GPIO_PIN_SET);
	pulse_enables_clock();
}
void e_16v_off(){
	HAL_GPIO_WritePin(GPIOC, P_16V_E, GPIO_PIN_RESET);
	pulse_enables_clock();
}
void e_16v_on(){
	HAL_GPIO_WritePin(GPIOC, P_16V_E, GPIO_PIN_SET);
	pulse_enables_clock();
}
void led_red_off(){
	HAL_GPIO_WritePin(GPIOB, P_RED_LED, GPIO_PIN_RESET);
}
void led_red_on(){
	HAL_GPIO_WritePin(GPIOB, P_RED_LED, GPIO_PIN_SET);
}

void led_green_off(){
	HAL_GPIO_WritePin(GPIOB, P_GREEN_LED, GPIO_PIN_RESET);
}
void led_green_on(){
	HAL_GPIO_WritePin(GPIOB, P_GREEN_LED, GPIO_PIN_SET);
}
void led_blue_off(){
	HAL_GPIO_WritePin(GPIOB, P_BLUE_LED, GPIO_PIN_RESET);
}
void led_blue_on(){
	HAL_GPIO_WritePin(GPIOB, P_BLUE_LED, GPIO_PIN_SET);
}

void set_enables_init(){
	e_parallel_on();
	HAL_Delay(1000);
	micropower_on(); //set parallel enable and micropower on outputs
	HAL_Delay(1000);
	if(!(HAL_GPIO_ReadPin(GPIOC, P_REEDSW_DET)))// when reed sw open, get high on detect pin,
		e_vbatt_on(); //thus only turn on vbatt if det pin low
	HAL_Delay(1000);
	e_5v_on();
	HAL_Delay(1000);
	e_12v_on();
	//e_16v_on();
	pulse_enables_clock(); //update 4 enables and send clock pulse to update
}

void set_cont_lights(){
	led_green_on();
	led_red_on();
	led_blue_on();
}

//ADC inputs request and transmission--------------------------------------------------------------------
void r_bat0_1(){
	adc_sel(ADC_CHANNEL_15);
	uint8_t adc_data = (uint8_t)adc_read();
	adc_reset_conf();
	write_txL_buffer(T_BAT0_1|(uint16_t)adc_data);
}
void r_bat0_2(){
	adc_sel(ADC_CHANNEL_14);
	uint8_t adc_data = (uint8_t)adc_read();
	adc_reset_conf();
	write_txL_buffer(T_BAT0_2|(uint16_t)adc_data);
}
void r_bat0_3(){
	adc_sel(ADC_CHANNEL_7);
	uint8_t adc_data = (uint8_t)adc_read();
	adc_reset_conf();
	write_txL_buffer((T_BAT0_3|(uint16_t)adc_data));
}
void r_bat0_4(){
	adc_sel(ADC_CHANNEL_6);
	uint8_t adc_data = (uint8_t)adc_read();
	adc_reset_conf();
	write_txL_buffer((T_BAT0_4|(uint16_t)adc_data));
}
void r_bat0_5(){
	adc_sel(ADC_CHANNEL_5);
	uint8_t adc_data = (uint8_t)adc_read();
	adc_reset_conf();
	write_txL_buffer((T_BAT0_5|(uint16_t)adc_data));
}
void r_bat0_6(){
	adc_sel(ADC_CHANNEL_4);
	uint8_t adc_data = (uint8_t)adc_read();
	adc_reset_conf();
	write_txL_buffer((T_BAT0_6|(uint16_t)adc_data));
}
void r_bat1_1(){
	adc_sel(ADC_CHANNEL_3);
	uint8_t adc_data = (uint8_t)adc_read();
	adc_reset_conf();
	write_txL_buffer((T_BAT1_1|(uint16_t)adc_data));
}
void r_bat1_2(){
	adc_sel(ADC_CHANNEL_2);
	uint8_t adc_data = (uint8_t)adc_read();
	adc_reset_conf();
	write_txL_buffer((T_BAT1_2|(uint16_t)adc_data));
}
void r_bat1_3(){
	adc_sel(ADC_CHANNEL_1);
	uint8_t adc_data = (uint8_t)adc_read();
	adc_reset_conf();
	write_txL_buffer((T_BAT1_3|(uint16_t)adc_data));
}
void r_bat1_4(){
	adc_sel(ADC_CHANNEL_0);
	uint8_t adc_data = (uint8_t)adc_read();
	adc_reset_conf();
	write_txL_buffer((T_BAT1_4|(uint16_t)adc_data));
}
void r_bat1_5(){
	adc_sel(ADC_CHANNEL_13);
	uint8_t adc_data = (uint8_t)adc_read();
	adc_reset_conf();
	write_txL_buffer((T_BAT1_5|(uint16_t)adc_data));
}
void r_bat1_6(){
	adc_sel(ADC_CHANNEL_12);
	uint8_t adc_data = (uint8_t)adc_read();
	adc_reset_conf();
	write_txL_buffer((T_BAT1_6|(uint16_t)adc_data));
}
void r_ext_pres(){
	adc_sel(ADC_CHANNEL_10);
	uint8_t adc_data = (uint8_t)adc_read();
	adc_reset_conf();
	write_txL_buffer((T_EXT_PRES|(uint16_t)adc_data));
}
void r_water_sense(){
	adc_sel(ADC_CHANNEL_11);
	uint8_t adc_data = (uint8_t)adc_read();
	adc_reset_conf();
	write_txL_buffer((T_WATER_SENSE|(uint16_t)adc_data));
}


//GPIO input pins Read and transmit-------------------------------------------------------------
void r_reed_det(){
	if (HAL_GPIO_ReadPin(GPIOC, P_REEDSW_DET))
		write_txL_buffer((T_REED_DET | 0x0001));
	else
		write_txL_buffer(T_REED_DET);
}

void r_auxreed1(){
	if (HAL_GPIO_ReadPin(GPIOB, P_AUXREED1))
		write_txL_buffer((T_AUXREED1 | 0x0001));
	else
		write_txL_buffer(T_AUXREED1);
}

void r_auxreed2(){
	if (HAL_GPIO_ReadPin(GPIOB, P_AUXREED2))
		write_txL_buffer((T_AUXREED2 | 0x0001));
	else
		write_txL_buffer(T_AUXREED2);
}

void r_auxreed3(){
	if (HAL_GPIO_ReadPin(GPIOB, P_AUXREED3))
		write_txL_buffer((T_AUXREED3 | 0x0001));
	else
		write_txL_buffer(T_AUXREED3);
}

void r_parallel_e(){
	if (HAL_GPIO_ReadPin(GPIOB, P_PARALLEL_E))
		write_txL_buffer((T_PARALLEL_E | 0x0001));
	else
		write_txL_buffer(T_PARALLEL_E);
}

void r_micropower(){
	if (HAL_GPIO_ReadPin(GPIOC, P_MICROPOWER))
		write_txL_buffer((T_MICROPOWER | 0x0001));
	else
		write_txL_buffer(T_MICROPOWER);
}
void r_vbatt_e(){
	if (HAL_GPIO_ReadPin(GPIOC, P_VBATT_E))
		write_txL_buffer((T_VBATT_E | 0x0001));
	else
		write_txL_buffer(T_VBATT_E);
}
void r_5v_e(){
	if (HAL_GPIO_ReadPin(GPIOC, P_5V_E))
		write_txL_buffer((T_5V_E | 0x0001));
	else
		write_txL_buffer(T_5V_E);
}
void r_12v_e(){
	if (HAL_GPIO_ReadPin(GPIOC, P_12V_E))
		write_txL_buffer((T_12V_E | 0x0001));
	else
		write_txL_buffer(T_12V_E);
}
void r_16v_e(){
	if (HAL_GPIO_ReadPin(GPIOC, P_16V_E))
		write_txL_buffer((T_16V_E | 0x0001));
	else
		write_txL_buffer(T_16V_E);
}
void r_red(){
	if (HAL_GPIO_ReadPin(GPIOB, P_RED_LED))
		write_txL_buffer((T_RED | 0x0001));
	else
		write_txL_buffer(T_RED);
}
void r_green(){
	if (HAL_GPIO_ReadPin(GPIOB, P_GREEN_LED))
		write_txL_buffer((T_GREEN | 0x0001));
	else
		write_txL_buffer(T_GREEN);
}
void r_blue(){
	if (HAL_GPIO_ReadPin(GPIOB, P_BLUE_LED))
		write_txL_buffer((T_BLUE | 0x0001));
	else
		write_txL_buffer(T_BLUE);
}

void r_all_io(){//transmits gpio input and gpio output pins data in following format
	//0xE000 | BLUE, GREEN, RED, 16VE, 12VE, 5VE, VBATT, MICROPOWER, PARALLEL_E, AUX3, AUX2, AUX1, REED_DET
	write_txL_buffer(T_ALL_IO|
			HAL_GPIO_ReadPin(GPIOB, P_BLUE_LED)<<12|
			HAL_GPIO_ReadPin(GPIOB, P_GREEN_LED)<<11|
			HAL_GPIO_ReadPin(GPIOB, P_RED_LED)<<10|
			HAL_GPIO_ReadPin(GPIOC, P_16V_E)<<9|
			HAL_GPIO_ReadPin(GPIOC, P_12V_E)<<8|
			HAL_GPIO_ReadPin(GPIOC, P_5V_E)<<7|
			HAL_GPIO_ReadPin(GPIOC, P_VBATT_E)<<6|
			HAL_GPIO_ReadPin(GPIOC, P_MICROPOWER)<<5|
			HAL_GPIO_ReadPin(GPIOB, P_PARALLEL_E)<<4|
			HAL_GPIO_ReadPin(GPIOB, P_AUXREED3)<<3|
			HAL_GPIO_ReadPin(GPIOB, P_AUXREED2)<<2|
			HAL_GPIO_ReadPin(GPIOB, P_AUXREED1)<<1|
			HAL_GPIO_ReadPin(GPIOC, P_REEDSW_DET));
}


//reed switch
void reed_switch_flipped(){// when reed switch flipped, sends state
	if (HAL_GPIO_ReadPin(GPIOC, P_REEDSW_DET)){ // if high, sw open = vbatt off, value 1 for high
		write_txL_buffer((T_REED_DET | 0x0001));
		e_vbatt_off();
	}else{
		write_txL_buffer(T_REED_DET);
		e_vbatt_on();
	}
}


//Buffer errors and bad rx instructions
void txL_buffer_full(){
	write_txH_buffer(TXL_BUFFER_FULL_ERROR);
}
void rx_buffer_full(){
	write_txH_buffer(RX_BUFFER_FULL_ERROR);
}
void bad_rx_request(){
	write_txH_buffer(INVALID_REQUEST_ERROR);
}
