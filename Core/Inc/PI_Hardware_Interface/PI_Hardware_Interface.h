/*
 * PI_Hardware_Interface.h
 *
 *  Created on: Mar 7, 2025
 *      Author: MARIO
 */

#ifndef INC_PI_HARDWARE_INTERFACE_PI_HARDWARE_INTERFACE_H_
#define INC_PI_HARDWARE_INTERFACE_PI_HARDWARE_INTERFACE_H_

#include "main.h"
#include "stm32f4xx_hal.h"
//your includes
#include "../../../Drivers/Motor_Control/PI/PI.h"
#include "../ModBus_RTU_Hardware_Interface/HMI_Modbus.h"

typedef enum
{
	TIMER_ERROR,
	TIMER_NOERROR

}Timer_Status_t;





//timer 6 used to call function "Evalute" for th PI
//called every 5ms or 200HZ
//ARR=4999 ,PSC=83
extern TIM_HandleTypeDef *htim;
//Get data from modbus
extern HoldingRegs_t *Hregs; //test
extern InputRegs_t *Iregs;

Timer_Status_t PI_Timer_start_HI(TIM_HandleTypeDef *htim);


//called when the overflows occurs calling the evalute function every 5ms
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)


#endif /* INC_PI_HARDWARE_INTERFACE_PI_HARDWARE_INTERFACE_H_ */
