/*
 * Utils_Hardware_Interface.c
 *
 *  Created on: Mar 13, 2025
 *      Author: MARIO
 */

#include "Utils_Hardware_Interface.h"

void HI_ApplyTimings(TIM_HandleTypeDef *htim, float tA, float tB, float tC)
{
	htim->Instance->CCR1 = (uint16_t) (tA * (float) TIM_1_8_PERIOD_CLOCKS);
	htim->Instance->CCR2 = (uint16_t) (tB * (float) TIM_1_8_PERIOD_CLOCKS);
	htim->Instance->CCR3 = (uint16_t) (tC * (float) TIM_1_8_PERIOD_CLOCKS);

}
int HI_SVM_Angle(float Amplitude, float Angle, float* tA, float* tB, float* tC)
{
	float Alpha, Beta;
	uint8_t Sextant;
	GetAlphaBeta(Amplitude, Angle, &Alpha, &Beta);
	int x = SVM(Alpha, Beta, &Sextant, tA, tB, tC);
	return x;
}

