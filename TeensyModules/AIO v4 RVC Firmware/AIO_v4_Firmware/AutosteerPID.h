/*****************************************************************************
 * @file        AutosteerPID.h
 *
 * @brief:      This  Module is for PID control of the steering motor
 *
 * @defgroup AutosteerPID Autosteer PID
 * @addtogroup modules
 * @ingroup common
 * @{
 * */

#ifndef __AUTOSTEER_PID_MODULE__
#define __AUTOSTEER_PID_MODULE__

/********************************************************************************
 * INCLUDE DIRECTIVES
 ********************************************************************************/
#include <stdint.h>

/********************************************************************************
 * DEFINITIONS, ENUMS, STRUCTURES AND TYPEDEFS
 ********************************************************************************/

// How many degrees before decreasing Max PWM
#define LOW_HIGH_DEGREES 3.0

// Variables for settings
typedef struct PID_Parameters
{
    uint8_t Kp;     // proportional gain
    uint8_t lowPWM; // band of no action
    int16_t wasOffset;
    uint8_t minPWM;
    uint8_t highPWM; // max PWM value
    float steerSensorCounts;
    float AckermanFix; // sent as percent

} PID_Parameters_t;

int16_t calcSteeringPID(PID_Parameters_t *pidParams, float pidError);

int16_t motorDrive(int16_t pwmDrive);


#endif //__AUTOSTEER_PID_MODULE__


