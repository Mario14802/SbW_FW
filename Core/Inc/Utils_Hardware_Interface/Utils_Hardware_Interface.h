/*
 * Utils_Hardware_Interface.h
 *
 *  Created on: Mar 13, 2025
 *      Author: MARIO
 */

#ifndef INC_UTILS_HARDWARE_INTERFACE_UTILS_HARDWARE_INTERFACE_H_
#define INC_UTILS_HARDWARE_INTERFACE_UTILS_HARDWARE_INTERFACE_H_

#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

#include "../../Drivers/Motor_Control/utils/utils.h"

#define TIM_1_8_PERIOD_CLOCKS 3500

TIM_HandleTypeDef *htim;


void HI_ApplyTimings(TIM_HandleTypeDef *htim, float tA, float tB, float tC);
int HI_SVM_Angle(float Amplitude, float Angle, float* tA, float* tB, float* tC);


#endif /* INC_UTILS_HARDWARE_INTERFACE_UTILS_HARDWARE_INTERFACE_H_ */
