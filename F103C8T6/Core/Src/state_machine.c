/*
 * state_machine.c
 *
 *  Created on: Oct 8, 2020
 *      Author: admin
 */


#include "state_machine.h"

STATE_t getState()
{
	return currState;
}

void setState(STATE_t st)
{
	currState = st;
}
