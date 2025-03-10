/*
 * PI_Hardware_Interface.c
 *
 *  Created on: Mar 7, 2025
 *      Author: MARIO
 */

#include "PI_Hardware_Interface.h"

//PI_Handle_t *PI;


Timer_Status_t PI_Timer_start_HI(TIM_HandleTypeDef *htim)
{
	if(HAL_TIM_Base_Start_IT(htim)!= HAL_OK){return TIMER_ERROR;}

	else{return TIMER_NOERROR;}
}



