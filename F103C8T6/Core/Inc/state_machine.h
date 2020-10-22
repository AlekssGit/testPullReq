/*
 * state_machine.h
 *
 *  Created on: Oct 8, 2020
 *      Author: admin
 */

#ifndef INC_STATE_MACHINE_H_
#define INC_STATE_MACHINE_H_

typedef enum {
	WORK_IBP,
	ERROR_IBP
} STATE_t;

STATE_t currState;

STATE_t getState();
void setState(STATE_t st);

#endif /* INC_STATE_MACHINE_H_ */
