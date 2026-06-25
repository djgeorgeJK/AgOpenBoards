
#include <stdint.h>
#include "AutosteerPID.h"
#include "Arduino.h"        //abs()
#include "gpio.h"           //MC_PWM_1, MC_PWM_2LEFT, MC_DIRECTION_PIN



int16_t calcSteeringPID(PID_Parameters_t *pidParams, float pidError)
{
    //Proportional only
    float pValue = pidParams->Kp * pidError;
    int16_t pwmDrive = (int16_t)pValue;

    float errorAbs = abs(pidError);
    int16_t newMax = 0;
    float highLowPerDeg = 0;
    // for PWM High to Low interpolator
    highLowPerDeg = ((float)(pidParams->highPWM - pidParams->lowPWM)) / LOW_HIGH_DEGREES;

    if (errorAbs < LOW_HIGH_DEGREES)
    {
        newMax = (errorAbs * highLowPerDeg) + pidParams->lowPWM;
    }
    else newMax = pidParams->highPWM;

    //add min throttle factor so no delay from motor resistance.
    if (pwmDrive < 0) pwmDrive -= pidParams->minPWM;
    else if (pwmDrive > 0) pwmDrive += pidParams->minPWM;

    //Serial.printf(newMax); //The actual steering angle in degrees
    //Serial.printf(",");

    //limit the pwm drive
    if (pwmDrive > newMax) pwmDrive = newMax;
    if (pwmDrive < -newMax) pwmDrive = -newMax;

    // if (pidParams->IsDanfoss)
    // {
    //     // Danfoss: PWM 25% On = Left Position max  (below Valve=Center)
    //     // Danfoss: PWM 50% On = Center Position
    //     // Danfoss: PWM 75% On = Right Position max (above Valve=Center)
    //     pwmDrive = (constrain(pwmDrive, -250, 250));

    //     // Calculations below make sure pwmDrive values are between 65 and 190
    //     // This means they are always positive, so in motorDrive, no need to check for
    //     // steerConfig.isDanfoss anymore
    //     pwmDrive = pwmDrive >> 2; // Devide by 4
    //     pwmDrive += 128;          // add Center Pos.

    //     // pwmDrive now lies in the range [65 ... 190], which would be great for an ideal opamp
    //     // However the TLC081IP is not ideal. Approximating from fig 4, 5 TI datasheet, @Vdd=12v, T=@40Celcius, 0 current
    //     // Voh=11.08 volts, Vol=0.185v
    //     // (11.08/12)*255=235.45
    //     // (0.185/12)*255=3.93
    //     // output now lies in the range [67 ... 205], the center position is now 136
    //     //pwmDrive = (map(pwmDrive, 4, 235, 0, 255));
    // }
    return pwmDrive;
}

//############################# Motor Low level ############################################################


int16_t motorDrive(int16_t pwmDrive)
{
    static int16_t motorLastPwm = -9999;
    bool isRight = pwmDrive > 0 ? true : false;

    if (motorLastPwm != pwmDrive)
    {
        motorLastPwm = pwmDrive;
        Serial.printf("PWM motoru: %d\r\n", pwmDrive);
    }

    // Used with Cytron MD30C Driver or BLDC
    // Steering Motor
    // Dir + PWM Signal

    // Cytron MD30C Driver Dir + PWM Signal
    digitalWrite(MC_DIRECTION_PIN, isRight);

    //write out the 0 to 255 value
    if (pwmDrive < 0)
    {
        pwmDrive = -1 * pwmDrive;
    }
    analogWrite(MC_PWM_1, pwmDrive);


    // {    Versio for some driver
    //     // IBT 2 Driver Dir1 connected to BOTH enables
    //     // PWM Left + PWM Right Signal
    //     if (pwmDrive > 0)
    //     {
    //         analogWrite(MC_PWM_2LEFT, 0);//Turn off before other one on
    //         analogWrite(MC_PWM_1, pwmDrive);
    //     }
    //     else
    //     {
    //         pwmDrive = -1 * pwmDrive;
    //         analogWrite(MC_PWM_1, 0);//Turn off before other one on
    //         analogWrite(MC_PWM_2LEFT, pwmDrive);
    //     }
    // }
    return pwmDrive;
}

