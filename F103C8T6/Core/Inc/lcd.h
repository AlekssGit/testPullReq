/*
 * lcd.h
 *
 *  Created on: Sep 22, 2020
 *      Author: alvas
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "main.h"

#define DP 0xFD

void print_digit(int dig, short int number, short int dp);
void print_number(int number);
void update_lcd();
void clear_lcd();
uint8_t prepare_number(short int number);


#endif /* INC_LCD_H_ */
