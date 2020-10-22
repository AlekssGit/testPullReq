/*
 * measurements.h
 *
 *  Created on: Oct 1, 2020
 *      Author: alvas
 */

#ifndef INC_MEASUREMENTS_H_
#define INC_MEASUREMENTS_H_

#include "stm32f1xx_hal.h"

#define FILTR_NUM 8
#define SIZE 4

#define TEMP_55_ADC 4073
#define U_0 		0
#define U_24 		2771	//2979
#define U_29 		3348    //3599
#define I_20 		2482

extern uint16_t adc_1[4];

extern uint16_t adc_U_ac_dc[FILTR_NUM];
extern uint16_t adc_U_AB[FILTR_NUM];
extern uint16_t adc_I_shunt[FILTR_NUM];
extern uint16_t adc_temp[FILTR_NUM];


extern uint32_t U_AB;
extern uint32_t U_ac_dc;
extern uint32_t I_shunt;
extern uint32_t temperature;

extern uint32_t filtr_U_ac_dc;
extern uint32_t filtr_U_AB;
extern uint32_t filtr_I_shunt;
extern uint32_t filtr_temperature;

int counter_steps;


void mean_vals();

void measure_data();

void init_filtr();


#endif /* INC_MEASUREMENTS_H_ */
