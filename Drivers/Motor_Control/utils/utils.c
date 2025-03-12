#include "utils.h"
#include <math.h>
#include <float.h>
#include <stm32f4xx_hal.h>

int SVM(float alpha, float beta, uint8_t *Sext, float* tA, float* tB, float* tC)
{
	int Sextant;
//	if(!alpha && !beta)
//	{
//		*tA = 0;
//		*tB = 0;
//		*tC = 0;
//		return 0;
//	}
	if (beta >= 0.0f)
	{
		if (alpha >= 0.0f)
		{
			//quadrant I
			if (one_by_sqrt3 * beta > alpha)
				Sextant = 2; //sextant v2-v3
			else
				Sextant = 1; //sextant v1-v2

		}
		else
		{
			//quadrant II
			if (-one_by_sqrt3 * beta > alpha)
				Sextant = 3; //sextant v3-v4
			else
				Sextant = 2; //sextant v2-v3
		}
	}
	else
	{
		if (alpha >= 0.0f)
		{
			//quadrant IV
			if (-one_by_sqrt3 * beta > alpha)
				Sextant = 5; //sextant v5-v6
			else
				Sextant = 6; //sextant v6-v1
		}
		else
		{
			//quadrant III
			if (one_by_sqrt3 * beta > alpha)
				Sextant = 4; //sextant v4-v5
			else
				Sextant = 5; //sextant v5-v6
		}
	}
	*Sext = Sextant;
	//a->CurrentSector = Sextant;
	//SetNextAdcChannels(a);
	switch (Sextant)
	{
		// sextant v1-v2
		case 1:
		{
			// Vector on-times
			float t1 = alpha - one_by_sqrt3 * beta;
			float t2 = two_by_sqrt3 * beta;

			// PWM timings
			*tA = (1.0f - t1 - t2) * 0.5f;
			*tB = *tA + t1;
			*tC = *tB + t2;
		}
			break;

			// sextant v2-v3
		case 2:
		{
			// Vector on-times
			float t2 = alpha + one_by_sqrt3 * beta;
			float t3 = -alpha + one_by_sqrt3 * beta;

			// PWM timings
			*tB = (1.0f - t2 - t3) * 0.5f;
			*tA = *tB + t3;
			*tC = *tA + t2;
		}
			break;

			// sextant v3-v4
		case 3:
		{
			// Vector on-times
			float t3 = two_by_sqrt3 * beta;
			float t4 = -alpha - one_by_sqrt3 * beta;

			// PWM timings
			*tB = (1.0f - t3 - t4) * 0.5f;
			*tC = *tB + t3;
			*tA = *tC + t4;
		}
			break;

			// sextant v4-v5
		case 4:
		{
			// Vector on-times
			float t4 = -alpha + one_by_sqrt3 * beta;
			float t5 = -two_by_sqrt3 * beta;

			// PWM timings
			*tC = (1.0f - t4 - t5) * 0.5f;
			*tB = *tC + t5;
			*tA = *tB + t4;
		}
			break;

			// sextant v5-v6
		case 5:
		{
			// Vector on-times
			float t5 = -alpha - one_by_sqrt3 * beta;
			float t6 = alpha - one_by_sqrt3 * beta;

			// PWM timings
			*tC = (1.0f - t5 - t6) * 0.5f;
			*tA = *tC + t5;
			*tB = *tA + t6;
		}
			break;

			// sextant v6-v1
		case 6:
		{
			// Vector on-times
			float t6 = -two_by_sqrt3 * beta;
			float t1 = alpha + one_by_sqrt3 * beta;

			// PWM timings
			*tA = (1.0f - t6 - t1) * 0.5f;
			*tC = *tA + t1;
			*tB = *tC + t6;
		}
			break;
	}

//	//Deadtime compenation = 49.5 out of 3500
//	// = 0.014142
//	if (a->Ia > 0) {
//		*tA += 0.014142;
//	} else {
//		*tA -= 0.014142;
//	}
//	if (a->Ib > 0) {
//		*tB += 0.014142;
//	} else {
//		*tB -= 0.014142;
//	}
//	if (a->Ic > 0) {
//		*tC += 0.014142;
//	} else {
//		*tC -= 0.014142;
//	}

	// if any of the results becomes NaN, result_valid will evaluate to false
	int result_valid = *tA >= 0.0f && *tA <= 1.0f && *tB >= 0.0f && *tB <= 1.0f && *tC >= 0.0f && *tC <= 1.0f;
	return result_valid ? 0 : -1;
}

//applies the timings into the PWM module
void ApplyTimings(TIM_HandleTypeDef *htim, float tA, float tB, float tC)
{
	htim->Instance->CCR1 = (uint16_t) (tA * (float) TIM_1_8_PERIOD_CLOCKS);
	htim->Instance->CCR2 = (uint16_t) (tB * (float) TIM_1_8_PERIOD_CLOCKS);
	htim->Instance->CCR3 = (uint16_t) (tC * (float) TIM_1_8_PERIOD_CLOCKS);
}

void GetAlphaBeta(float Mag, float Angle, float *Alpha, float *Beta)
{
	*Alpha = Mag * our_arm_cos_f32(Angle);
	*Beta = Mag * our_arm_sin_f32(Angle);
}

/*
 * Valpha = Mag Cos(Theta)
 * Vbeta = Mag Sin(Theta)
 * Mag Q1.15
 * Angle Q1.15
 *
 */
Volt_Components GetAlphaBetaFp(int16_t Mag, int16_t Angle)
{
	Volt_Components AlphaBeta;
	Trig_Components SinCos = MCM_Trig_Functions(Angle);
	int32_t Alpha, Beta;
	Alpha = ((int32_t) Mag * SinCos.hCos) / 32768;
	Beta = ((int32_t) Mag * SinCos.hSin) / 32768;
	AlphaBeta.qV_Component1 = (int16_t) Alpha;
	AlphaBeta.qV_Component2 = (int16_t) Beta;
	return AlphaBeta;
}

// based on https://math.stackexchange.com/a/1105038/81278
float fast_atan2(float y, float x)
{
	// a := min (|x|, |y|) / max (|x|, |y|)
	float abs_y = fabsf(y);
	float abs_x = fabsf(x);
	// inject FLT_MIN in denominator to avoid division by zero
	float a = MACRO_MIN(abs_x, abs_y) / (MACRO_MAX(abs_x, abs_y) + FLT_MIN);
	// s := a * a
	float s = a * a;
	// r := ((-0.0464964749 * s + 0.15931422) * s - 0.327622764) * s * a + a
	float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
	// if |y| > |x| then r := 1.57079637 - r
	if (abs_y > abs_x)
		r = 1.57079637f - r;
	// if x < 0 then r := 3.14159274 - r
	if (x < 0.0f)
		r = 3.14159274f - r;
	// if y < 0 then r := -r
	if (y < 0.0f)
		r = -r;

	return r;
}

// Evaluate polynomials using Fused Multiply Add intrisic instruction.
// coeffs[0] is highest order, as per numpy.polyfit
// p(x) = coeffs[0] * x^deg + ... + coeffs[deg], for some degree "deg"
float horner_fma(float x, const float *coeffs, size_t count)
{
	float result = 0.0f;
	for (int idx = 0; idx < count; ++idx)
		result = fmaf(result, x, coeffs[idx]);
	return result;
}

// Modulo (as opposed to remainder), per https://stackoverflow.com/a/19288271
int mod(int dividend, int divisor)
{
	int r = dividend % divisor;
	return (r < 0) ? (r + divisor) : r;
}

/*
 // @brief: Returns how much time is left until the deadline is reached.
 // If the deadline has already passed, the return value is 0 (except if
 // the deadline is very far in the past)
 uint32_t deadline_to_timeout(uint32_t deadline_ms) {
 uint32_t now_ms = (uint32_t)((1000ull * (uint64_t)osKernelSysTick()) / osKernelSysTickFrequency);
 uint32_t timeout_ms = deadline_ms - now_ms;
 return (timeout_ms & 0x80000000) ? 0 : timeout_ms;
 }
 */

/*
 // @brief: Converts a timeout to a deadline based on the current time.
 uint32_t timeout_to_deadline(uint32_t timeout_ms) {
 uint32_t now_ms = (uint32_t)((1000ull * (uint64_t)osKernelSysTick()) / osKernelSysTickFrequency);
 return now_ms + timeout_ms;
 }
 */

/*
 // @brief: Returns a non-zero value if the specified system time (in ms)
 // is in the future or 0 otherwise.
 // If the time lies far in the past this may falsely return a non-zero value.
 int is_in_the_future(uint32_t time_ms) {
 return deadline_to_timeout(time_ms);
 }
 */

/*
 // @brief: Returns number of microseconds since system startup
 uint32_t micros(void) {
 register uint32_t ms, cycle_cnt;
 do {
 ms = HAL_GetTick();
 cycle_cnt = TIM_TIME_BASE->CNT;
 } while (ms != HAL_GetTick());

 return (ms * 1000) + cycle_cnt;
 }*/

int SVM_Angle(float Amplitude, float Angle, float* tA, float* tB, float* tC)
{
	float Alpha, Beta;
	uint8_t Sextant;
	GetAlphaBeta(Amplitude, Angle, &Alpha, &Beta);
	int x = SVM(Alpha, Beta, &Sextant, tA, tB, tC);
	return x;
}

#define SQRT3FACTOR (uint16_t) 0xDDB4 /* = (16384 * 1.732051 * 2)*/

int16_t hT_Sqrt3 = ((TIM_1_8_PERIOD_CLOCKS) * SQRT3FACTOR ) / 16384u;
;
uint16_t hSector;

uint16_t PWMC_SetPhaseVoltage(Volt_Components AlphaBeta, uint16_t *tA, uint16_t *tB, uint16_t *tC)
{
	int32_t wX, wY, wZ, wUAlpha, wUBeta, wTimePhA, wTimePhB, wTimePhC;
	//PWMC_SetSampPointSectX_Cb_t pSetADCSamplingPoint;

	wUAlpha = AlphaBeta.qV_Component1 * (int32_t) hT_Sqrt3;
	wUBeta = -(AlphaBeta.qV_Component2 * (int32_t) ( TIM_1_8_PERIOD_CLOCKS)) * 2;

	wX = wUBeta;
	wY = (wUBeta + wUAlpha) / 2;
	wZ = (wUBeta - wUAlpha) / 2;

	/* Sector calculation from wX, wY, wZ */
	if (wY < 0)
	{
		if (wZ < 0)
		{
			hSector = SECTOR_5;
			wTimePhA = (int32_t) ( TIM_1_8_PERIOD_CLOCKS) / 4 + ((wY - wZ) / (int32_t) 262144);
			wTimePhB = wTimePhA + wZ / 131072;
			wTimePhC = wTimePhA - wY / 131072;
			//pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect5;
		}
		else /* wZ >= 0 */
		if (wX <= 0)
		{
			hSector = SECTOR_4;
			wTimePhA = (int32_t) ( TIM_1_8_PERIOD_CLOCKS) / 4 + ((wX - wZ) / (int32_t) 262144);
			wTimePhB = wTimePhA + wZ / 131072;
			wTimePhC = wTimePhB - wX / 131072;
			//pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect4;
		}
		else /* wX > 0 */
		{
			hSector = SECTOR_3;
			wTimePhA = (int32_t) ( TIM_1_8_PERIOD_CLOCKS) / 4 + ((wY - wX) / (int32_t) 262144);
			wTimePhC = wTimePhA - wY / 131072;
			wTimePhB = wTimePhC + wX / 131072;
			//pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect3;
		}
	}
	else /* wY > 0 */
	{
		if (wZ >= 0)
		{
			hSector = SECTOR_2;
			wTimePhA = (int32_t) ( TIM_1_8_PERIOD_CLOCKS) / 4 + ((wY - wZ) / (int32_t) 262144);
			wTimePhB = wTimePhA + wZ / 131072;
			wTimePhC = wTimePhA - wY / 131072;
			//pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect2;
		}
		else /* wZ < 0 */
		if (wX <= 0)
		{
			hSector = SECTOR_6;
			wTimePhA = (int32_t) ( TIM_1_8_PERIOD_CLOCKS) / 4 + ((wY - wX) / (int32_t) 262144);
			wTimePhC = wTimePhA - wY / 131072;
			wTimePhB = wTimePhC + wX / 131072;
			//pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect6;
		}
		else /* wX > 0 */
		{
			hSector = SECTOR_1;
			wTimePhA = (int32_t) ( TIM_1_8_PERIOD_CLOCKS) / 4 + ((wX - wZ) / (int32_t) 262144);
			wTimePhB = wTimePhA + wZ / 131072;
			wTimePhC = wTimePhB - wX / 131072;
			//pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect1;
		}
	}

	*tA = (uint16_t) wTimePhA;
	*tB = (uint16_t) wTimePhB;
	*tC = (uint16_t) wTimePhC;
	return 0;
//  pHandle->hCntPhA = ( uint16_t )wTimePhA;
//  pHandle->hCntPhB = ( uint16_t )wTimePhB;
//  pHandle->hCntPhC = ( uint16_t )wTimePhC;
//
//  if ( pHandle->DTTest == 1u )
//  {
//    /* Dead time compensation */
//    if ( pHandle->hIa > 0 )
//    {
//      pHandle->hCntPhA += pHandle->DTCompCnt;
//    }
//    else
//    {
//      pHandle->hCntPhA -= pHandle->DTCompCnt;
//    }
//
//    if ( pHandle->hIb > 0 )
//    {
//      pHandle->hCntPhB += pHandle->DTCompCnt;
//    }
//    else
//    {
//      pHandle->hCntPhB -= pHandle->DTCompCnt;
//    }
//
//    if ( pHandle->hIc > 0 )
//    {
//      pHandle->hCntPhC += pHandle->DTCompCnt;
//    }
//    else
//    {
//      pHandle->hCntPhC -= pHandle->DTCompCnt;
//    }
//  }
//
//  return ( pSetADCSamplingPoint( pHandle ) );
}

// @brief: Busy wait delay for given amount of microseconds (us)
void delay_us(uint32_t us)
{
	uint32_t start = micros();
	while (micros() - start < (uint32_t) us)
	{
		__ASM("nop");
	}
}

/**
 * Fast sine and cosine implementation.
 *
 * See http://lab.polygonal.de/?p=205
 *
 * @param angle
 * The angle in radians
 * WARNING: Don't use too large angles.
 *
 * @param sin
 * A pointer to store the sine value.
 *
 * @param cos
 * A pointer to store the cosine value.
 */
void utils_fast_sincos_better(float angle, float *sin, float *cos)
{
	//always wrap input angle to -PI..PI
	while (angle < -M_PI)
	{
		angle += 2.0f * M_PI;
	}

	while (angle > M_PI)
	{
		angle -= 2.0f * M_PI;
	}

	//compute sine
	if (angle < 0.0)
	{
		*sin = 1.27323954f * angle + 0.405284735f * angle * angle;

		if (*sin < 0.0)
		{
			*sin = 0.225f * (*sin * -*sin - *sin) + *sin;
		}
		else
		{
			*sin = 0.225f * (*sin * *sin - *sin) + *sin;
		}
	}
	else
	{
		*sin = 1.27323954f * angle - 0.405284735 * angle * angle;

		if (*sin < 0.0)
		{
			*sin = 0.225f * (*sin * -*sin - *sin) + *sin;
		}
		else
		{
			*sin = 0.225f * (*sin * *sin - *sin) + *sin;
		}
	}

	// compute cosine: sin(x + PI/2) = cos(x)
	angle += 0.5f * M_PI;
	if (angle > M_PI)
	{
		angle -= 2.0f * M_PI;
	}

	if (angle < 0.0)
	{
		*cos = 1.27323954f * angle + 0.405284735f * angle * angle;

		if (*cos < 0.0)
		{
			*cos = 0.225f * (*cos * -*cos - *cos) + *cos;
		}
		else
		{
			*cos = 0.225f * (*cos * *cos - *cos) + *cos;
		}
	}
	else
	{
		*cos = 1.27323954f * angle - 0.405284735f * angle * angle;

		if (*cos < 0.0)
		{
			*cos = 0.225f * (*cos * -*cos - *cos) + *cos;
		}
		else
		{
			*cos = 0.225f * (*cos * *cos - *cos) + *cos;
		}
	}
}
