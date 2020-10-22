/*
 * test.c
 *
 *  Created on: Sep 24, 2020
 *      Author: alvas
 */

#include "test.h"


void all_leds_on()
{
	HAL_GPIO_WritePin(U_AB_GPIO_Port, U_AB_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(U_ac_dc_GPIO_Port, U_ac_dc_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(U_AB_29_GPIO_Port, U_AB_29_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(U_AB_24_GPIO_Port, U_AB_24_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(U_ac_dc_0_GPIO_Port, U_ac_dc_0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(T_high_GPIO_Port, T_high_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(I_mor_20_GPIO_Port, I_mor_20_Pin, GPIO_PIN_SET);
}
void all_leds_off()
{
	HAL_GPIO_WritePin(U_AB_GPIO_Port, U_AB_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(U_ac_dc_GPIO_Port, U_ac_dc_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(U_AB_29_GPIO_Port, U_AB_29_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(U_AB_24_GPIO_Port, U_AB_24_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(U_ac_dc_0_GPIO_Port, U_ac_dc_0_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(T_high_GPIO_Port, T_high_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(I_mor_20_GPIO_Port, I_mor_20_Pin, GPIO_PIN_RESET);
}
void all_segments_on()
{
	HAL_GPIO_WritePin(GPIOB, MR_LCD_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(dig_1_GPIO_Port, dig_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(dig_2_GPIO_Port, dig_2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(dig_3_GPIO_Port, dig_3_Pin, GPIO_PIN_SET);


}
void all_segments_off()
{
	HAL_GPIO_WritePin(GPIOB, MR_LCD_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(dig_1_GPIO_Port, dig_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(dig_2_GPIO_Port, dig_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(dig_3_GPIO_Port, dig_3_Pin, GPIO_PIN_RESET);
}
