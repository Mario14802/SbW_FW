#include "FOC.h"
#include <math.h>  // For sqrt()

// =======================
// Clark Transform Function
// =======================
Motor_Error clark_transform(PhaseCurrents* input_currents, Alpha_Beta* Alpha_BetaC)
{
    if (!input_currents || !Alpha_BetaC) {
        return MOTOR_ERROR_INVALID_INPUT; // Return error if inputs are invalid
    }

    // Extract the currents from the input struct
    float Ia = input_currents->currents[0];
    float Ib = input_currents->currents[1];

    // Perform the Clark transformation (alpha, beta)
    Alpha_BetaC->alpha = Ia;
    Alpha_BetaC->beta = one_by_sqrt3 * (Ia + (2 * Ib)); // I_beta = 1/sqrt(3) * (I_a + 2*I_b)

    return MOTOR_ERROR_NONE;
}

// =======================
// Space Vector PWM (SVPWM) Function
// =======================
void SVPWM(float alpha, float beta, Pwm_TIME *Time)
{
    if (Time == NULL) {
        // Handle the error if Time pointer is invalid
        return; // Or log error and return an error code
    }

    uint32_t Sextant;
    float tA, tB, tC;

    // Quadrant I: 0° to 90° (sectors 1 and 2)
    if (alpha >= 0.0f && beta >= 0.0f) {
        if (one_by_sqrt3 * beta > alpha) {
            Sextant = 2; // Sextant v2-v3
        } else {
            Sextant = 1; // Sextant v1-v2
        }
    }
    // Quadrant II: 90° to 180° (sectors 2 and 3)
    else if (alpha < 0.0f && beta >= 0.0f) {
        if (-one_by_sqrt3 * beta > alpha) {
            Sextant = 3; // Sextant v3-v4
        } else {
            Sextant = 2; // Sextant v2-v3
        }
    }
    // Quadrant III: 180° to 270° (sectors 4 and 5)
    else if (alpha < 0.0f && beta < 0.0f) {
        if (one_by_sqrt3 * beta > alpha) {
            Sextant = 4; // Sextant v4-v5
        } else {
            Sextant = 5; // Sextant v5-v6
        }
    }
    // Quadrant IV: 270° to 360° (sectors 5 and 6)
    else if (alpha >= 0.0f && beta < 0.0f) {
        if (-one_by_sqrt3 * beta > alpha) {
            Sextant = 5; // Sextant v5-v6
        } else {
            Sextant = 6; // Sextant v6-v1
        }
    }

    switch (Sextant) {
        case 1: {
            // Vector on-times for sector 1
            float t1 = alpha - one_by_sqrt3 * beta;
            float t2 = two_by_sqrt3 * beta;

            // PWM timings
            tA = (1.0f - t1 - t2) * 0.5f;
            tB = tA + t1;
            tC = tB + t2;
        } break;

        case 2: {
            // Vector on-times for sector 2
            float t2 = alpha + one_by_sqrt3 * beta;
            float t3 = -alpha + one_by_sqrt3 * beta;

            // PWM timings
            tB = (1.0f - t2 - t3) * 0.5f;
            tA = tB + t3;
            tC = tA + t2;
        } break;

        case 3: {
            // Vector on-times for sector 3
            float t3 = two_by_sqrt3 * beta;
            float t4 = -alpha - one_by_sqrt3 * beta;

            // PWM timings
            tB = (1.0f - t3 - t4) * 0.5f;
            tC = tB + t3;
            tA = tC + t4;
        } break;

        case 4: {
            // Vector on-times for sector 4
            float t4 = -alpha + one_by_sqrt3 * beta;
            float t5 = -two_by_sqrt3 * beta;

            // PWM timings
            tC = (1.0f - t4 - t5) * 0.5f;
            tB = tC + t5;
            tA = tB + t4;
        } break;

        case 5: {
            // Vector on-times for sector 5
            float t5 = -alpha - one_by_sqrt3 * beta;
            float t6 = alpha - one_by_sqrt3 * beta;

            // PWM timings
            tC = (1.0f - t5 - t6) * 0.5f;
            tA = tC + t5;
            tB = tA + t6;
        } break;

        case 6: {
            // Vector on-times for sector 6
            float t6 = -two_by_sqrt3 * beta;
            float t1 = alpha + one_by_sqrt3 * beta;

            // PWM timings
            tA = (1.0f - t6 - t1) * 0.5f;
            tC = tA + t1;
            tB = tC + t6;
        } break;
    }

    // Assign the calculated PWM timings to the Time structure
    Time->Ta = tA;
    Time->Tb = tB;
    Time->Tc = tC;
}
