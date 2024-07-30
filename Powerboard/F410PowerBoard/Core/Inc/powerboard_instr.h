/*
 * PowerBoard_Directives.h
 *
 *  Created on: Mar 22, 2024
 *      Author: calum
 */

#ifndef INC_POWERBOARD_DIRECTIVES_H_
#define INC_POWERBOARD_DIRECTIVES_H_
#endif /* INC_POWERBOARD_DIRECTIVES_H_ */


//output instructions- all toggles so 1/0 for on off
#define E_PARALLEL_0 0x11
#define E_PARALLEL_1 0x12
#define MICROPOWER_0 0x13
#define MICROPOWER_1 0x14
#define E_VBATT_0 0x15
#define E_VBATT_1 0x16
#define E_5V_0 0x17
#define E_5V_1 0x18
#define E_12V_0 0x19
#define E_12V_1 0x1A
#define E_16V_0 0x1B
#define E_16V_1 0x1C
#define RED_0 0x1D
#define RED_1 0x1E
#define GREEN_0 0x1F
#define GREEN_1 0x20
#define BLUE_0 0x21
#define BLUE_1 0x22


//data requests, requires R at end of instruction

//ADC requests

#define R_BAT0_1 0x30
#define R_BAT0_2 0x31
#define R_BAT0_3 0x32
#define R_BAT0_4 0x33
#define R_BAT0_5 0x34
#define R_BAT0_6 0x35
#define R_BAT1_1 0x36
#define R_BAT1_2 0x37
#define R_BAT1_3 0x38
#define R_BAT1_4 0x39
#define R_BAT1_5 0x3A
#define R_BAT1_6 0x3B
#define R_EXT_PRES 0x3C
#define R_WATER_SENSE 0x3D


// gpio input requests
#define R_REED_DET 0x40
#define R_AUXREED1 0x41
#define R_AUXREED2 0x42
#define R_AUXREED3 0x43



//request gpio output information
#define R_PARALLEL_E 0x44
#define R_MICROPOWER 0x45
#define R_VBATT_E 0x46
#define R_5V_E 0x47
#define R_12V_E 0x48
#define R_16V_E 0x49
#define R_RED 0x4A
#define R_GREEN 0x4B
#define R_BLUE 0x4C
#define R_ALL_IO 0x4D // not analog


//transmission format for alerts and requests.
//alerts will be 0xFFF + alert number
//reading data from gpio output pins will be 0xFE+pin identifier+  1/0 -  7 bit identifier is possible
//adc will be 1 byte indicator starting from 00+ downconverted 8bit adc
//cant use 12 bit as only leaves 16 other transmissions we can send with 15
//adcs - we have 9 non-indicator gpios and 8 alerts

//alerts
#define CA_BAT0 0xFFF0
#define CA_BAT1 0xFFF1
#define CA_PARALLEL 0xFFF2
#define CA_VBATT 0xFFF3
#define CA_5V 0xFFF4
#define CA_12V 0xFFF5
#define CA_16V 0xFFF6
#define TXL_BUFFER_FULL_ERROR 0xFFFD
#define RX_BUFFER_FULL_ERROR 0xFFFE
#define INVALID_REQUEST_ERROR 0xFFFF


// GPIO pins
//inputs
#define T_REED_DET 0xFE00  // code adds last char 0/1
#define T_AUXREED1 0xFE10
#define T_AUXREED2 0xFE20
#define T_AUXREED3 0xFE30

//outputs
#define T_PARALLEL_E 0xFE40
#define T_MICROPOWER 0xFE50
#define T_VBATT_E 0xFE60
#define T_5V_E 0xFE70
#define T_12V_E 0xFE80
#define T_16V_E 0xFE90
#define T_RED 0xFEA0
#define T_GREEN 0xFEB0
#define T_BLUE 0xFEC0

#define T_ALL_IO 0x2000

//ADC transmissions
#define T_BAT0_1 0x1000
#define T_BAT0_2 0x1100
#define T_BAT0_3 0x1200
#define T_BAT0_4 0x1300
#define T_BAT0_5 0x1400
#define T_BAT0_6 0x1500
#define T_BAT1_1 0x1700
#define T_BAT1_2 0x1800
#define T_BAT1_3 0x1900
#define T_BAT1_4 0x1A00
#define T_BAT1_5 0x1B00
#define T_BAT1_6 0x1C00
#define T_EXT_PRES 0x1D00
#define T_WATER_SENSE 0x1E00

//instruction functions in .c file
void pulse_enables_clock(void);

void e_parallel_off(void);
void e_parallel_on(void);
void micropower_off(void);
void micropower_on(void);
void e_vbatt_off(void);
void e_vbatt_on(void);
void e_5v_off(void);
void e_5v_on(void);
void e_12v_off(void);
void e_12v_on(void);
void e_16v_off(void);
void e_16v_on(void);
void led_red_on(void);
void led_red_off(void);
void led_green_off(void);
void led_green_on(void);
void led_blue_off(void);
void led_blue_on(void);

void set_enables_init(void);
void set_cont_lights(void);

void r_bat0_1(void);
void r_bat0_2(void);
void r_bat0_3(void);
void r_bat0_4(void);
void r_bat0_5(void);
void r_bat0_6(void);
void r_bat1_1(void);
void r_bat1_2(void);
void r_bat1_3(void);
void r_bat1_4(void);
void r_bat1_5(void);
void r_bat1_6(void);
void r_ext_pres(void);
void r_water_sense(void);

void r_reed_det(void);
void r_auxreed1(void);
void r_auxreed2(void);
void r_auxreed3(void);
void r_parallel_e(void);
void r_micropower(void);
void r_vbatt_e(void);
void r_5v_e(void);
void r_12v_e(void);
void r_16v_e(void);
void r_red(void);
void r_green(void);
void r_blue(void);
void r_all_io(void);

void txL_buffer_full(void);
void rx_buffer_full(void);
void bad_rx_request(void);


