typedef struct
{
    float Kp;
    float lowPWM;
    float highPWM;
    float minPWM;
} pidPar_t;

void calcSteeringPID(float pidError)
{
    //Proportional only
    pValue = steerSettings.Kp * pidError;
    pwmDrive = (int16_t)pValue;

    errorAbs = abs(pidError);
    int16_t newMax = 0;

    if (errorAbs < LOW_HIGH_DEGREES)
    {
        newMax = (errorAbs * highLowPerDeg) + steerSettings.lowPWM;
    }
    else newMax = steerSettings.highPWM;

    //add min throttle factor so no delay from motor resistance.
    if (pwmDrive < 0) pwmDrive -= steerSettings.minPWM;
    else if (pwmDrive > 0) pwmDrive += steerSettings.minPWM;

    //Serial.printf(newMax); //The actual steering angle in degrees
    //Serial.printf(",");

    //limit the pwm drive
    if (pwmDrive > newMax) pwmDrive = newMax;
    if (pwmDrive < -newMax) pwmDrive = -newMax;

    if (steerConfig.MotorDriveDirection) pwmDrive *= -1;

    if (steerConfig.IsDanfoss)
    {
        // Danfoss: PWM 25% On = Left Position max  (below Valve=Center)
        // Danfoss: PWM 50% On = Center Position
        // Danfoss: PWM 75% On = Right Position max (above Valve=Center)
        pwmDrive = (constrain(pwmDrive, -250, 250));

        // Calculations below make sure pwmDrive values are between 65 and 190
        // This means they are always positive, so in motorDrive, no need to check for
        // steerConfig.isDanfoss anymore
        pwmDrive = pwmDrive >> 2; // Devide by 4
        pwmDrive += 128;          // add Center Pos.

        // pwmDrive now lies in the range [65 ... 190], which would be great for an ideal opamp
        // However the TLC081IP is not ideal. Approximating from fig 4, 5 TI datasheet, @Vdd=12v, T=@40Celcius, 0 current
        // Voh=11.08 volts, Vol=0.185v
        // (11.08/12)*255=235.45
        // (0.185/12)*255=3.93
        // output now lies in the range [67 ... 205], the center position is now 136
        //pwmDrive = (map(pwmDrive, 4, 235, 0, 255));
    }
}

//############################# Motor Low level ############################################################


void motorDrive(void)
{
    static int16_t motorLastPwm = -9999;
    bool isRight = pwmDrive > 0 ? true : false;
    if (steerConfig.IsRelayActiveHigh)
    {
        isRight = !isRight; // if relay is active high, the direction is reversed.
    }

    if (motorLastPwm != pwmDrive)
    {
        motorLastPwm = pwmDrive;
        Serial.printf("PWM motoru: %d\r\n", pwmDrive);
    }


    // Used with Cytron MD30C Driver
    // Steering Motor
    // Dir + PWM Signal
    if (steerConfig.CytronDriver)
    {
        // Cytron MD30C Driver Dir + PWM Signal
        digitalWrite(MC_DIRECTION_PIN, isRight);

        //write out the 0 to 255 value
        if (pwmDrive < 0)
        {
            pwmDrive = -1 * pwmDrive;
        }
        analogWrite(MC_PWM_1, pwmDrive);
    }
    else
    {
        // IBT 2 Driver Dir1 connected to BOTH enables
        // PWM Left + PWM Right Signal
        if (pwmDrive > 0)
        {
            analogWrite(MC_PWM_2LEFT, 0);//Turn off before other one on
            analogWrite(MC_PWM_1, pwmDrive);
        }
        else
        {
            pwmDrive = -1 * pwmDrive;
            analogWrite(MC_PWM_1, 0);//Turn off before other one on
            analogWrite(MC_PWM_2LEFT, pwmDrive);
        }
    }
    pwmDisplay = pwmDrive;
}

