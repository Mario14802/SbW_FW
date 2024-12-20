#ifndef FOC_H
#define FOC_H


#include <stdint.h>

// =======================
// Constants
// =======================
//#define ONE_BY_SQRT3 (1.0f / sqrt(3.0f)) // Precomputed constant 1/sqrt(3)

// Math Constants

#define  one_by_sqrt3  		 0.57735026919f
#define  two_by_sqrt3		 1.15470053838f
#define  sqrt3_by_2 		 0.86602540378f

// =======================
// Structs
// =======================
typedef struct {
    float currents[2]; // Two-phase system currents (alpha and beta axes)
} PhaseCurrents;

typedef struct {
    float alpha;  // Alpha axis (Direct axis)
    float beta;   // Beta axis (Quadrature axis)
} Alpha_Beta;

// =======================
// Error Enumeration
// =======================
typedef enum {
    MOTOR_ERROR_NONE = 0,       // No error
    MOTOR_ERROR_INVALID_INPUT   // Invalid input error
} Motor_Error;


typedef struct
{
	float Ta;
	float Tb;
	float Tc;

}Pwm_TIME;


// =======================
// Function Prototypes
// =======================
Motor_Error clark_transform(PhaseCurrents* input_currents, Alpha_Beta* Alpha_BetaC);

void SVPWM(float alpha, float beta, Pwm_TIME *Time);



#endif // FOC_H
