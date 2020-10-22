/*
 * lcd.c
 *
 *  Created on: Sep 24, 2020
 *      Author: alvas
 */

#include "lcd.h"

extern uint32_t U_AB;
extern uint32_t U_ac_dc;
extern uint32_t temperature;
extern int val;
extern int change;

extern SPI_HandleTypeDef hspi2;

int displayed_number = -1;

void update_lcd()
{
	if(change == 1)
	{
		change = 0;

		HAL_GPIO_WritePin(dig_1_GPIO_Port, dig_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dig_2_GPIO_Port, dig_2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dig_3_GPIO_Port, dig_3_Pin, GPIO_PIN_RESET);

		HAL_Delay(500);
	}

	if(val == 1)
	{
		print_number(U_AB);
	}
	else
	{
		print_number(U_ac_dc);
//		print_number(temperature);
	}
}


void print_number(int number)
{
	if(number < 10)
	{
		print_digit(2, 0, 1);
		HAL_Delay(1);
		print_digit(3, number, 0);
		HAL_Delay(1);
	}
	else if(number < 100)
	{
		print_digit(2, number/10, 1);
		HAL_Delay(1);
		print_digit(3, number % 10, 0);
		HAL_Delay(1);
	}
	else if(number < 1000)
	{
		print_digit(1, number / 100, 0);
		HAL_Delay(1);
		print_digit(2, (number % 100) / 10, 1);
		HAL_Delay(1);
		print_digit(3, (number % 100) % 10, 0);
		HAL_Delay(1);
	}
}

void print_digit(int dig, short int number, short int dp)
{
	switch(dig)
	{
	case 1:
			HAL_GPIO_WritePin(dig_3_GPIO_Port, dig_3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(dig_1_GPIO_Port, dig_1_Pin, GPIO_PIN_SET);
			break;
	case 2:
			HAL_GPIO_WritePin(dig_3_GPIO_Port, dig_3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(dig_1_GPIO_Port, dig_1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(dig_2_GPIO_Port, dig_2_Pin, GPIO_PIN_SET);
			break;
	case 3:
			HAL_GPIO_WritePin(dig_1_GPIO_Port, dig_1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(dig_2_GPIO_Port, dig_2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(dig_3_GPIO_Port, dig_3_Pin, GPIO_PIN_SET);
			break;
	}

	uint8_t aTxBuffer[1] = {0};
	aTxBuffer[0] = prepare_number(number);

	if(dp) aTxBuffer[0] &= DP;

	HAL_SPI_Transmit(&hspi2, (uint8_t*)aTxBuffer, 1, 0);
}

uint8_t prepare_number(short int number)
{
	uint8_t res;
	switch(number)
	{
	case 0: res = 0x0A; break;
	case 1: res = 0xEB; break;
	case 2: res = 0x26; break;
	case 3: res = 0xA2; break;
	case 4: res = 0xC3; break;
	case 5: res = 0x92; break;
	case 6: res = 0x12; break;
	case 7: res = 0xAB; break;
	case 8: res = 0x02; break;
	case 9: res = 0x82; break;
	}
	return res;
}
