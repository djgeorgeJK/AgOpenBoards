/*
   UDP Autosteer code for Teensy 4.1
   For AgOpenGPS
   01 Feb 2022
   Like all Arduino code - copied from somewhere else :)
   So don't claim it as your own
*/

#include "AutosteerPID.h"
////////////////// User Settings /////////////////////////



/*  PWM Frequency ->
     490hz (default) = 0
     122hz = 1
     3921hz = 2
*/
#define PWM_Frequency 2000

// WAS Calabration
typedef enum
{
    WAS_50 = 0,
    WAS_45,
    WAS_40,
    WAS_35,
    WAS_30,
    WAS_25,
    WAS_20,
    WAS_15,
    WAS_10,
    WAS_5,
    WAS_0,
    WAS5,
    WAS10,
    WAS15,
    WAS20,
    WAS25,
    WAS30,
    WAS35,
    WAS40,
    WAS45,
    WAS50
} eWAS;
float inputWAS[] = {-50.00, -45.0, -40.0, -35.0, -30.0, -25.0, -20.0, -15.0, -10.0, -5.0, 0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0}; // Input WAS do not adjust
float outputWAS[] = {-50.00, -45.0, -40.0, -35.0, -30.0, -25.0, -20.0, -15.0, -10.0, -5.0, 0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0};

/////////////////////////////////////////////

//   ***********  Motor drive connections  **************
// Connect ground only for cytron, Connect Ground and +5v for IBT2

// see GPIO in dpio.h for pin definitions




// Not Connected for Cytron, Right PWM for IBT2
#define MC_PWM_2LEFT        4

#define CONST_180_DIVIDED_BY_PI 57.2957795130823

#include <Wire.h>
#include <EEPROM.h>
#include "zADS1115.h"
#include "Machine_UDP.h"
#include "global.h"
#include "Buttons.h"

#ifdef USE_EXTERN_ADC
    ADS1115_lite adc(ADS1115_DEFAULT_ADDRESS); // Use this for the 16-bit version ADS1115
#endif

#include <IPAddress.h>

// ethernet

extern IPAddress Eth_ipDestination;
extern byte Eth_myip[];
extern unsigned int portDestination;

// uint8_t Ethernet::buffer[200]; // udp send and receive buffer
uint8_t autoSteerUdpData[UDP_TX_PACKET_MAX_SIZE]; // Buffer For Receiving UDP Data

// loop time variables in mili-seconds
const uint16_t LOOP_TIME = 25; // 25ms - 40Hz
uint32_t autsteerLastTime = LOOP_TIME;
uint32_t currentTime = LOOP_TIME;

const uint16_t WATCHDOG_THRESHOLD = 100;
const uint16_t WATCHDOG_FORCE_VALUE = WATCHDOG_THRESHOLD + 2; // Should be greater than WATCHDOG_THRESHOLD
uint8_t watchdogTimer = WATCHDOG_FORCE_VALUE;
#define Watchdog_Reset()    watchdogTimer = 0;

// Heart beat hello AgIO
uint8_t helloFromIMU[] = {128, 129, 121, 121, 5, 1, 2, 3, 4, 5, 71};
                      // 0x80,0x81,0x79,0x79,0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47
uint8_t helloFromAutoSteer[] = {0x80, 0x81, 126, 126, 5, 0, 0, 0, 0, 0, 71}; // bytes 5-9 are data
int16_t helloSteerPosition = 0;

//uint8_t helloFromMachine[] = {128, 129, 123, 123, 5, 0, 0, 0, 0, 0, 71};
uint8_t helloFromMachine[] = {0x80, 0x81, 0x7B, 0x7B, 0x05, 0x00, 0x00, FW_MAJ, FW_MIN, FW_PATCH, 0x47};
// fromAutoSteerData FD 253 - ActualSteerAngle*100 -5,6, SwitchByte-7, pwmDisplay-8

uint8_t PGN_253[] = {0x80, 0x81, 126, 0xFD, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC};
int8_t PGN_253_Size = sizeof(PGN_253) - 1;

// fromAutoSteerData FD 250 - sensor values etc
uint8_t PGN_250[] = {0x80, 0x81, 126, 0xFA, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC};
int8_t PGN_250_Size = sizeof(PGN_250) - 1;
uint8_t aog2Count = 0;
float sensorReading;
float sensorSample;
elapsedMillis gpsSpeedUpdateTimer = 0;

///////////   Function protptypes  /////////////

#ifdef ARDUINO_TEENSY41
void SendUdp(uint8_t *data, uint8_t datalen, IPAddress dip, uint16_t dport);
#endif

// EEPROM
int16_t EEread = 0;

// Relays
uint8_t relay = 0, relayHi = 0, uTurn = 0;
uint8_t tram = 0;

// Switches
Switches_t ButtState;

// On Off
uint8_t guidanceStatus = 0;
uint8_t prevGuidanceStatus = 0;
bool guidanceStatusChanged = false;

// speed sent as *10
float gpsSpeed = 0;

// steering variables
float steerAngleActual = 0;
float steerAngleSetPoint = 0; // the desired angle from AgOpen
float steerAngleError = 0;    // setpoint - actual

// pwm variables
int16_t pwmDisplay = 0;


// Steer switch button  ***********************************************************************************************************
uint8_t pulseCount = 0; // Steering Wheel Encoder
bool encEnable = false; // debounce flag
uint8_t thisEnc = 0, lastEnc = 0;

// Variables for settings
//Storage steerSettings; // 11 bytes
PID_Parameters_t steerSettings = {
    .Kp = 40,               // proportional gain
    .lowPWM = 10,           // band of no action
    .wasOffset = 0,
    .minPWM = 9,
    .highPWM = 60,          // max PWM value
    .steerSensorCounts = 30,
    .AckermanFix = 1        // sent as percent
};


// Variables for settings - 0 is false
typedef struct SteerConfig
{
    bool InvertWAS = 0;             // Byte 0, Bit 0 - Wheel angle Sensor
    bool IsRelayActiveHigh = 0;     // Byte 0, Bit 1 - if zero, active low (default)
    bool MotorDriveDirection = 0;   // Byte 0, Bit 2 - Motor Drive Direction
    bool SingleInputWAS = 1;        // Byte 0, Bit 3 - Single Input WAS
    bool CytronDriver = 1;          // Byte 0, Bit 4 - Cytron Driver
    bool SteerSwitch = 0;           // Byte 0, Bit 5 - 1 if switch selected
    bool SteerButton = 0;           // Byte 0, Bit 6 - 1 if button selected
    bool ShaftEncoder = 0;          // Byte 0, Bit 7 - 1 if shaft encoder selected
    uint8_t PulseCountMax = 5;      // Byte 1
    bool IsDanfoss = 0;             // Byte 2, Bit 0
    bool PressureSensor = 0;        // Byte 2, Bit 1
    bool CurrentSensor = 0;         // Byte 2, Bit 2
    uint8_t IsUseY_Axis = 1;        // Byte 2, Bit 3 - Set to 0 to use X Axis, 1 to use Y Axis
}SteerConfig_t;
SteerConfig_t steerConfig;  // also saved in EE_ADDR_STEECFG

void steerConfigInit()
{
    if (0 == steerConfig.CytronDriver)
    {
        pinMode(MC_PWM_2LEFT, OUTPUT);
    }
}



void autosteerSetup()
{
    // init AD converter
    analog_init();
    analogReadRes(12);
    analogReadAveraging(16);
    pinMode(AN_POT_MY, INPUT_DISABLE);
    // init PWM for motor control
    analogWriteFrequency(MC_PWM_1, PWM_Frequency); // Zakladni nosna frekvence PWM
    analogWrite(MC_PWM_1, 0);                      // Start with 0% Duty
    // analogWriteFrequency(MC_PWM_2LEFT, PWM_Frequency);

    // keep pulled high and drag low to activate, noise free safe
    pinMode(MC_DIRECTION_PIN, OUTPUT);
    digitalWrite(MC_DIRECTION_PIN, 1); // default to high

    pinMode(WORKSW_PIN, INPUT_PULLUP);
    pinMode(STEERSW_PIN, INPUT_PULLUP);
    pinMode(REMOTE_PIN, INPUT_PULLUP);

    // Disable digital inputs for analog input pins
    pinMode(CURRENT_SENSOR_PIN, INPUT_DISABLE);
    pinMode(PRESSURE_SENSOR_PIN, INPUT_DISABLE);

  // set up communication
  #ifdef USE_EXTERN_ADC
    Wire1.end();
    Wire1.begin();

    // Check ADC
    if (adc.testConnection())
    {
        Serial.println("ADC Connecton OK");
    }
    else
    {
        Serial.println("ADC Connecton FAILED!");
        Autosteer_running = false;
    }
  #endif

    // 50Khz I2C
    // TWBR = 144;   //Is this needed?

    EEPROM.get(EE_ADDR_READY, EEread); // read identifier

    if (EEread != EEP_Ident) // check on first start and write EEPROM
    {
        EEPROM.put(EE_ADDR_READY, EEP_Ident);
        EEPROM.put(EE_ADDR_STEERSET, steerSettings);
        EEPROM.put(EE_ADDR_STEECFG, steerConfig);
        EEPROM.put(EE_ADDR_NETWORK, networkAddress);
        Serial.printf(" Autosetup EEPROM rewritten \r\n");
    }
    else
    {
        EEPROM.get(EE_ADDR_STEERSET, steerSettings); // read the Settings
        EEPROM.get(EE_ADDR_STEECFG, steerConfig);
        EEPROM.get(EE_ADDR_NETWORK, networkAddress);
        Serial.printf(" Autosetup EEPROM OK \r\n");
    }

    steerConfigInit();

    if (Autosteer_running)
    {
        Serial.println("Autosteer running, waiting for AgOpenGPS");
        // Autosteer Led goes Red if ADS1115 is found
        digitalWrite(AUTOSTEER_ACTIVE_LED, 0);
        digitalWrite(AUTOSTEER_STANDBY_LED, 1);
    }
    else
    {
        Autosteer_running = false; // Turn off auto steer if no ethernet (Maybe running T4.0)
        //    if(!Ethernet_running)Serial.println("Ethernet not available");
        Serial.println("Autosteer disabled, GPS only mode");
        return;
    }

  #ifdef USE_EXTERN_ADC
    adc.setSampleRate(ADS1115_REG_CONFIG_DR_128SPS); // 128 samples per second
    adc.setGain(ADS1115_REG_CONFIG_PGA_6_144V);
  #endif
} // End of Setup

// ********************  Main loop  *************************************************

void autosteerLoop()    // called from main loop
{
    static uint8_t lastSwitchByte = 0xFF;

    // Loop triggers every 25 msec and sends back gyro heading, and roll, steer angle etc
    currentTime = systick_millis_count;
    if (currentTime - autsteerLastTime < LOOP_TIME)
    {
        return;
    }
    autsteerLastTime = currentTime;

    // reset debounce
    encEnable = true;

    // If connection lost to AgOpenGPS, the watchdog will count up and turn off steering
    // good value is 0-100
    if (watchdogTimer++ > 250)
    {
        watchdogTimer = WATCHDOG_FORCE_VALUE;
        ButtState.steerSwitch = 1; // reset values like it turned off
        ButtState.currentState = 1;
    }

    // read all the switches
    ButtState.workSwitch = (bool)digitalRead(WORKSW_PIN); // read work switch

    // Engage steering via 1 PCB Button or 2 Tablet
    ButtState.reading = digitalRead(STEERSW_PIN);

    if (steerConfig.SteerSwitch == 1)
    {
        // Switch is off so reset ready for next switch on
        if (ButtState.reading == HIGH)
        {
            // Serial.printf("Sterr active\r\n");
            ButtState.currentState = 1;
            ButtState.steerSwitch = 1;
            ButtState.previous = ButtState.reading;
        }
    }

    // 2 Has tablet button been pressed?
    if (guidanceStatusChanged)
    {
        if (guidanceStatus == 1) // Must have changed Off >> On
        {
            ButtState.currentState = 0;
            ButtState.steerSwitch = 0;
        }
    }

    // If AOG has stopped steering, wait then turn off steerswitch ready for next engage.
    static int switchCounter = 0;

    if (ButtState.steerSwitch == 0 && guidanceStatus == 0)
    {
        if (switchCounter++ > 30)
        {
            ButtState.currentState = 1;
            ButtState.steerSwitch = 1;
        }
    }
    else
    {
        switchCounter = 0;
    }

    // Arduino software button code
    if (ButtState.reading == LOW && ButtState.previous == HIGH)
    {
        if (ButtState.currentState == 1)
        {
            ButtState.currentState = 0;
            ButtState.steerSwitch = 0;
        }
        else
        {
            ButtState.currentState = 1;
            ButtState.steerSwitch = 1;
        }
    }
    ButtState.previous = ButtState.reading;

    // Encoder sensor?
    if (steerConfig.ShaftEncoder && pulseCount >= steerConfig.PulseCountMax)
    {
        ButtState.steerSwitch = 1;
        ButtState.currentState = 1;
        ButtState.previous = 0;
    }

    // Pressure sensor?
    if (steerConfig.PressureSensor)
    {
        sensorSample = (float)analogRead(PRESSURE_SENSOR_PIN);
        sensorSample *= 0.25;
        sensorReading = sensorReading * 0.6 + sensorSample * 0.4;
        if (sensorReading >= steerConfig.PulseCountMax)
        {
            ButtState.steerSwitch = 1;
            ButtState.currentState = 1;
            ButtState.previous = 0;
        }
    }

    // Current sensor?
    if (steerConfig.CurrentSensor)
    {
        sensorSample = (float)analogRead(CURRENT_SENSOR_PIN);
        sensorSample = (abs(775 - sensorSample)) * 0.5;
        sensorReading = sensorReading * 0.7 + sensorSample * 0.3;
        sensorReading = min(sensorReading, 255);

        if (sensorReading >= steerConfig.PulseCountMax)
        {
            ButtState.steerSwitch = 1;
            ButtState.currentState = 1;
            ButtState.previous = 0;
        }
    }

    ButtState.remoteSwitch = digitalRead(REMOTE_PIN);
    uint8_t udpSwitchMask = 0;
    udpSwitchMask |= ((ButtState.workSwitch == false) || (CanBus_IsSeedingActive() != false)) ? ((uint8_t)RS_WORKING) : 0u;
    // only debug udpSwitchMask |= ((CanBus_IsSeedingActive() != false)) ? ((uint8_t)RS_WORKING) : 0u;
    udpSwitchMask |= (ButtState.remoteSwitch != 0) ? (RS_REMOTE_SWITCH) : 0; // put remote in bit 2
    udpSwitchMask |= (ButtState.steerSwitch != 0) ? (RS_STEERING) : 0;       // put steerswitch status in bit 1 position

    ButtState.switchByte = udpSwitchMask;

    if (udpSwitchMask != lastSwitchByte)
    {
        Serial.printf("UDP switch packet:");
        Serial.printf("Working : %s", udpSwitchMask & RS_WORKING ? "On" : "Off");
        Serial.printf("Steer: %s", udpSwitchMask & RS_REMOTE_SWITCH ? "On" : "Off");
        Serial.printf("Steer2: %s", udpSwitchMask & RS_STEERING ? "On" : "Off");
        Serial.printf("\r\n");
        lastSwitchByte = udpSwitchMask;
    }
// get steering position
    int16_t steerADC;
#ifdef USE_EXTERN_ADC
    if (steerConfig.SingleInputWAS) // Single Input ADS
    {
        adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);
        steerADC = adc.getConversion();     // 16 bit value 0-65535
        adc.triggerConversion();                    // ADS1115 Single Mode
        steerADC = (steerADC >> 1); // bit shift by 2  0 to 13610 is 0 to 5v
        helloSteerPosition = steerADC - 6800;
        steerADC = (steerADC - 6805;
    }
    else // ADS1115 Differential Mode
    {
        adc.setMux(ADS1115_REG_CONFIG_MUX_DIFF_0_1);
        steerADC = adc.getConversion();
        adc.triggerConversion();
        steerADC = (steerADC >> 1); // bit shift by 2  0 to 13610 is 0 to 5v
        helloSteerPosition = steerADC - 6800;
        steerADC = (steerADC - 6805;
    }
  #else
    steerADC = analogRead(AN_POT_MY); // I have 12bit
    //steerADC = steerADC >> 2;             // bit shift by 2 to get 10 bit value
                                          //  (0-1024) for 65°.
                                          // 1024/65 = 15.75 counts per degree.
                                          // So 32 counts is about 2 degrees of steer angle.
                                          // I have it set up so that the middle of the pot is zero, so subtract 512 to get -512 to +512 with zero in the middle. You will need to adjust this for your setup. You can also use a different pot and adjust the code accordingly.
    //steerADC -= (1024 / 2);           // get middle
  #endif

    // DETERMINE ACTUAL STEERING POSITION in Angles
    // convert position to steer angle. Use calculated XX counts per degree.
    //   ***** make sure that negative steer angle makes a left turn and positive value is a right turn *****
    if (steerConfig.InvertWAS)
    {
        steerADC = (steerADC - steerSettings.wasOffset);    // was Zero from AgOpenPS
        steerAngleActual = (float)(steerADC) / -steerSettings.steerSensorCounts;
    }
    else
    {
        steerADC = (steerADC + steerSettings.wasOffset);
        steerAngleActual = (float)(steerADC) / steerSettings.steerSensorCounts;
    }

    // Ackerman fix
    if (steerAngleActual < 0)
        steerAngleActual = (steerAngleActual * steerSettings.AckermanFix);

    // WAS fault or over 25km, cut steering
    if ((steerAngleActual < inputWAS[0]) || (steerAngleActual > inputWAS[20]) || gpsSpeed > 25)
    {
        ButtState.steerSwitch = 1; // reset values like it turned off
        ButtState.currentState = 1;
        ButtState.previous = 0;
        watchdogTimer = WATCHDOG_FORCE_VALUE;
        // Serial.printf("Zastavuji Steering");
    }

    // Map WAS - linear interpolation
    float mappedWAS = multiMap<float>(steerAngleActual, inputWAS, outputWAS, 21);
    steerAngleActual = mappedWAS;

    // static int16_t lastSteerADC = 0;
    // static float lastSteerAngle = 0;
    // if(lastSteerADC != steerADC )//|| abs(lastSteerAngle - steerAngleActual) > 0.2)
    // {
    //     Serial.printf("Steer ADC: %d, Steer Angle: %f\r\n", steerADC, steerAngleActual);
    //     lastSteerADC = steerADC;
    //     lastSteerAngle = steerAngleActual;
    // }

    if (watchdogTimer < WATCHDOG_THRESHOLD) // normal situation when Packet are receiving from AgOpenGPS.
    {
        //Serial.printf("Steer Actual: %f, Steer Setpoint: %f, GPS Speed: %f\r\n", steerAngleActual, steerAngleSetPoint, gpsSpeed);

        steerAngleError = steerAngleActual - steerAngleSetPoint; // calculate the steering error
        // if (abs(steerAngleError)< steerSettings.lowPWM) steerAngleError = 0;

        // Don't turn wheels if speed less than 0.3km/hr
        // if (gpsSpeed < 0.2)
        //     steerAngleError = 0;

        int16_t pwmDrive = calcSteeringPID(&steerSettings, steerAngleError); // do the pid
        if (steerConfig.MotorDriveDirection) pwmDrive *= -1;
        pwmDisplay = motorDrive(pwmDrive);      // out to motors the pwm value
        // Autosteer Led goes GREEN if autosteering

        digitalWrite(AUTOSTEER_ACTIVE_LED, 1);
        digitalWrite(AUTOSTEER_STANDBY_LED, 0);
    }
    else
    {
        // we've lost the comm to AgOpenGPS, or just stop request

        pwmDisplay = motorDrive(0); // out to motors the pwm value
        pulseCount = 0;
        // Autosteer Led goes back to RED when autosteering is stopped
        digitalWrite(AUTOSTEER_STANDBY_LED, 1);
        digitalWrite(AUTOSTEER_ACTIVE_LED, 0);
    }

} // end of main loop

void gps_speed(void)
{
    // Speed pulse
    if (gpsSpeedUpdateTimer < 1000)
    {
        if (speedPulseUpdateTimer > 200) // 100 (10hz) seems to cause tone lock ups occasionally
        {
            speedPulseUpdateTimer = 0;

            // 130 pp meter, 3.6 kmh = 1 m/sec = 130hz or gpsSpeed * 130/3.6 or gpsSpeed * 36.1111
            // gpsSpeed = ((float)(autoSteerUdpData[5] | autoSteerUdpData[6] << 8)) * 0.1;
            float speedPulse = gpsSpeed * 36.1111;

            // Serial.printf(gpsSpeed); Serial.printf(" -> "); Serial.println(speedPulse);

            if (gpsSpeed > 0.11)
            { // 0.10 wasn't high enough
                tone(GP_VELOCITY_PIN, uint16_t(speedPulse));
            }
            else
            {
                noTone(GP_VELOCITY_PIN);
            }
        }
    }
    else // if gpsSpeedUpdateTimer hasn't update for 1000 ms, turn off speed pulse
    {
        noTone(GP_VELOCITY_PIN);
    }

    if (encEnable)
    {
        thisEnc = digitalRead(REMOTE_PIN);
        if (thisEnc != lastEnc)
        {
            lastEnc = thisEnc;
            if (lastEnc)
                EncoderFunc();
        }
    }
}

extern EthernetUDP Eth_udpAutoSteer;
extern bool useBNO08xRVC;
#define UDP_AGIO_HEADER_B0B1         0x80, 0x81     // header for byte 0 and 1
#define UDP_MACHINE_HEADER_B3        0x7B           // byte 3 for machine data
// UDP Receive and transmit
/* Charakterizace dat kdyz prijdou
     Bajty   0  1  2  3  4  5  6  7  8  9 10 11 12`
 *   Header 80 81 7f
 *                   FE - 254 asi PGN paket
 *                   FC - 252 Steer settings - struct steerSettings
 *                   FB - 251 Steer Config   - struct steerConfig
 *                   CB - 200  Hello from AgIO  ++ dota z PC na hello
 *                              kdyz jsou aktovni ostatni moduly tak posle:   helloFromAutoSteer, helloFromIMU     ,helloFromMachine
 *                   v odpovedi je ale pak byte 2
 *                     7F - 127 Hello from machine
 *                      7E - 126 Hello from autosteer
 *                      7D - 125 Hello from IMU - sent on power up
 *                      7C - 124 Hello from GPS - sent on power up
 *                   CE - 201 - networkAddress
 *                      - 239 - Machine_ProcessData
 *                      - 238 - Machine_ProcessConfig
 *                      - 236 - Machine_ProcessRelayConfig
 *                      - 202 - whoami
 *                   7B - 123 Machine data - sent from AOG every 10ms, PGN 239 in AOG
 *                   7A - 122 Steer Data 2 - sent from autosteer to AOG, PGN 250 in AOG, for pressure sensor, current sensor, etc
 *
 *  *               */
void ReceiveUdp()
{
    uint16_t len = Eth_udpAutoSteer.parsePacket();

    // if (len > 0)
    // {
    //  Serial.printf("ReceiveUdp: ");
    //  Serial.println(len);
    // }

    // Check for len > 4, because we check byte 0, 1, 2 and 3
    if (len <= 3)
    {
        return ;
    }

    Eth_udpAutoSteer.read(autoSteerUdpData, UDP_TX_PACKET_MAX_SIZE);
    //Serial.printf("Ethernet data received.\r\n");

    if (autoSteerUdpData[0] == 0x80 && autoSteerUdpData[1] == 0x81 && autoSteerUdpData[2] == 0x7F) // Data 0x80,0x81,0x7F
    {
        if (autoSteerUdpData[3] == 0xFE && Autosteer_running) // 254
        {
            gpsSpeed = ((float)(autoSteerUdpData[5] | autoSteerUdpData[6] << 8)) * 0.1;
            gpsSpeedUpdateTimer = 0;

            prevGuidanceStatus = guidanceStatus;

            guidanceStatus = autoSteerUdpData[7];
            guidanceStatusChanged = (guidanceStatus != prevGuidanceStatus);

            // Bit 8,9    set point steer angle * 100 is sent
            steerAngleSetPoint = ((float)(autoSteerUdpData[8] | ((int8_t)autoSteerUdpData[9]) << 8)) * 0.01; // high low bytes

            // Serial.printf("steerAngleSetPoint: ");
            // Serial.println(steerAngleSetPoint);

            // Serial.println(gpsSpeed);
            if ((bitRead(guidanceStatus, 0) == 0) /* || (gpsSpeed < 0.1)*/ || (ButtState.steerSwitch == 1))
            {
                watchdogTimer = WATCHDOG_FORCE_VALUE; // turn off steering motor
            }
            else // valid conditions to turn on autosteer
            {
                watchdogTimer = 0; // reset watchdog
            }

            // Bit 10 Tram
            tram = autoSteerUdpData[10];

            // Bit 11
            relay = autoSteerUdpData[11];

            // Bit 12
            relayHi = autoSteerUdpData[12];

            //----------------------------------------------------------------------------
            // Serial Send to agopenGPS

            int16_t sa = (int16_t)(steerAngleActual * 100);
            //Serial.printf("Steer Angle PGN: %f\r\n", steerAngleActual);

            PGN_253[5] = (uint8_t)sa;
            PGN_253[6] = sa >> 8;

            // heading
            PGN_253[7] = (uint8_t)9999;
            PGN_253[8] = 9999 >> 8;

            // roll
            PGN_253[9] = (uint8_t)8888;
            PGN_253[10] = 8888 >> 8;

            PGN_253[11] = ButtState.switchByte;
            PGN_253[12] = (uint8_t)pwmDisplay;

            // checksum
            int16_t CK_A = 0;
            for (uint8_t i = 2; i < PGN_253_Size; i++)
                CK_A = (CK_A + PGN_253[i]);

            PGN_253[PGN_253_Size] = CK_A;

            // off to AOG
            SendUdp(PGN_253, sizeof(PGN_253), Eth_ipDestination, portDestination);

            // Steer Data 2 -------------------------------------------------
            if (steerConfig.PressureSensor || steerConfig.CurrentSensor)
            {
                if (aog2Count++ > 2)
                {
                    // Send fromAutosteer2
                    PGN_250[5] = (byte)sensorReading;

                    // add the checksum for AOG2
                    CK_A = 0;

                    for (uint8_t i = 2; i < PGN_250_Size; i++)
                    {
                        CK_A = (CK_A + PGN_250[i]);
                    }

                    PGN_250[PGN_250_Size] = CK_A;

                    // off to AOG
                    SendUdp(PGN_250, sizeof(PGN_250), Eth_ipDestination, portDestination);
                    aog2Count = 0;
                }
            }

            // Serial.println(steerAngleActual);
            //--------------------------------------------------------------------------
        }

        // steer settings
        else if (autoSteerUdpData[3] == 0xFC && Autosteer_running) // 252
        {
            // PID values
            steerSettings.Kp = ((float)autoSteerUdpData[5]); // read Kp from AgOpenGPS

            steerSettings.highPWM = autoSteerUdpData[6]; // read high pwm

            steerSettings.lowPWM = (float)autoSteerUdpData[7]; // read lowPWM from AgOpenGPS

            steerSettings.minPWM = autoSteerUdpData[8]; // read the minimum amount of PWM for instant on

            float temp = (float)steerSettings.minPWM * 1.2;
            steerSettings.lowPWM = (byte)temp;

            steerSettings.steerSensorCounts = autoSteerUdpData[9]; // sent as setting displayed in AOG

            steerSettings.wasOffset = (autoSteerUdpData[10]); // read was zero offset Lo

            steerSettings.wasOffset |= (autoSteerUdpData[11] << 8); // read was zero offset Hi

            steerSettings.AckermanFix = (float)autoSteerUdpData[12] * 0.01;

            Serial.printf("Stearsettnigs data new counts %f\r\n", steerSettings.steerSensorCounts);
            Serial.printf("steerSettings.wasOffset new counts %d\r\n", steerSettings.wasOffset);
            Serial.printf(" Kp %d \r\n", steerSettings.Kp);
            Serial.printf(" lowPWM %d \r\n", steerSettings.lowPWM);
            Serial.printf(" minPWM %d \r\n", steerSettings.minPWM);
            Serial.printf(" highPWM %d\r\n", steerSettings.highPWM);

            // crc
            // autoSteerUdpData[13];

            // store in EEPROM
            EEPROM.put(EE_ADDR_READY, EEP_Ident);
            EEPROM.put(EE_ADDR_STEERSET, steerSettings);
        }

        else if (autoSteerUdpData[3] == 0xFB) // 251 FB - SteerConfig
        {
            uint8_t sett = autoSteerUdpData[5]; // Config Data Byte 0

            Serial.printf("Steer config received, %d, Encoder %d, %d, %d\r\n", autoSteerUdpData[5], autoSteerUdpData[6], autoSteerUdpData[7], autoSteerUdpData[8]);
            if (bitRead(sett, 0)) steerConfig.InvertWAS = 1;           else steerConfig.InvertWAS = 0; // AG invert WAS
            if (bitRead(sett, 1)) steerConfig.IsRelayActiveHigh = 1;   else  steerConfig.IsRelayActiveHigh = 0;
            if (bitRead(sett, 2)) steerConfig.MotorDriveDirection = 1; else  steerConfig.MotorDriveDirection = 0; // AG Invert Motor Direction
            if (bitRead(sett, 3)) steerConfig.SingleInputWAS = 1;      else  steerConfig.SingleInputWAS = 0;
            if (bitRead(sett, 4)) steerConfig.CytronDriver = 1;        else  steerConfig.CytronDriver = 0;
            if (bitRead(sett, 5)) steerConfig.SteerSwitch = 1;         else  steerConfig.SteerSwitch = 0;
            if (bitRead(sett, 6)) steerConfig.SteerButton = 1;         else  steerConfig.SteerButton = 0;
            if (bitRead(sett, 7)) steerConfig.ShaftEncoder = 1;        else  steerConfig.ShaftEncoder = 0; // AG setting je to Turn Sensor

            steerConfig.PulseCountMax = autoSteerUdpData[6]; // Config Data Byte 1

            // was speed
            // autoSteerUdpData[7];

            sett = autoSteerUdpData[8]; // // Config Data Byte 2
            if (bitRead(sett, 0)) steerConfig.IsDanfoss = 1;      else steerConfig.IsDanfoss = 0;
            if (bitRead(sett, 1)) steerConfig.PressureSensor = 1; else steerConfig.PressureSensor = 0;
            if (bitRead(sett, 2)) steerConfig.CurrentSensor = 1;  else steerConfig.CurrentSensor = 0;
            if (bitRead(sett, 3)) steerConfig.IsUseY_Axis = 1;    else steerConfig.IsUseY_Axis = 0;

            // crc
            // autoSteerUdpData[13];

            EEPROM.put(EE_ADDR_STEECFG, steerConfig);

            // Re-Init
            steerConfigInit();

        } // end FB
        else if (autoSteerUdpData[3] == 200) // Hello from AgIO, reply with module if exist
        {
            if (Autosteer_running)
            {
                int16_t sa = (int16_t)(steerAngleActual * 100);

                helloFromAutoSteer[5] = (uint8_t)sa;
                helloFromAutoSteer[6] = sa >> 8;

                helloFromAutoSteer[7] = (uint8_t)helloSteerPosition;
                helloFromAutoSteer[8] = helloSteerPosition >> 8;
                helloFromAutoSteer[9] = ButtState.switchByte;

                SendUdp(helloFromAutoSteer, sizeof(helloFromAutoSteer), Eth_ipDestination, portDestination);
            }
            if (useBNO08xRVC)
            {
                SendUdp(helloFromIMU, sizeof(helloFromIMU), Eth_ipDestination, portDestination);
            }

            if (useMachine)
            {
                SendUdp(helloFromMachine, sizeof(helloFromMachine), Eth_ipDestination, portDestination);
            }
        }

        else if (autoSteerUdpData[3] == 201)
        {
            // make really sure this is the subnet pgn
            if (autoSteerUdpData[4] == 5 && autoSteerUdpData[5] == 201 && autoSteerUdpData[6] == 201)
            {
                networkAddress.ipOne = autoSteerUdpData[7];
                networkAddress.ipTwo = autoSteerUdpData[8];
                networkAddress.ipThree = autoSteerUdpData[9];

                // save in EEPROM and restart
                EEPROM.put(EE_ADDR_NETWORK, networkAddress);
                SCB_AIRCR = 0x05FA0004; // Teensy Reset
            }
        } // end 201

        else if (autoSteerUdpData[3] == 239) // Machine data
        {
            Machine_ProcessData(&autoSteerUdpData[0]);
        }
        else if (autoSteerUdpData[3] == 238) // Machine config PGN - 238 - EE
        {
            Machine_ProcessConfig(&autoSteerUdpData[0]);
        }
        else if (autoSteerUdpData[3] == 236) // EC Relay Pin Settings  relayConfig PGN - 236 - EC
        {
            Machine_ProcessRelayConfig(&autoSteerUdpData[0]);
        }
        // whoami
        else if (autoSteerUdpData[3] == 202)
        {
            // make really sure this is the reply pgn
            if (autoSteerUdpData[4] == 3 && autoSteerUdpData[5] == 202 && autoSteerUdpData[6] == 202)
            {
                IPAddress rem_ip = Eth_udpAutoSteer.remoteIP();

                // hello from AgIO
                uint8_t scanReply[] = {128, 129, Eth_myip[3], 203, 7,
                                       Eth_myip[0], Eth_myip[1], Eth_myip[2], Eth_myip[3],
                                       rem_ip[0], rem_ip[1], rem_ip[2], 23};

                // checksum
                int16_t CK_A = 0;
                for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++)
                {
                    CK_A = (CK_A + scanReply[i]);
                }
                scanReply[sizeof(scanReply) - 1] = CK_A;

                static IPAddress ipDest(255, 255, 255, 255);
                uint16_t portDest = 9999; // AOG port that listens

                // off to AOG
                SendUdp(scanReply, sizeof(scanReply), ipDest, portDest);
            }
        }
    } // end if 80 81 7F
}

/****************************************************************************************************/
/**                         Local functions                                                        **/
/****************************************************************************************************/

#ifdef ARDUINO_TEENSY41
void SendUdp(uint8_t *data, uint8_t datalen, IPAddress dip, uint16_t dport)
{
    Eth_udpAutoSteer.beginPacket(dip, dport);
    Eth_udpAutoSteer.write(data, datalen);
    Eth_udpAutoSteer.endPacket();
}
#endif

// ISR Steering Wheel Encoder
void EncoderFunc()
{
    if (encEnable)
    {
        pulseCount++;
        encEnable = false;
    }
}

// Rob Tillaart, https://github.com/RobTillaart/MultiMap
template<typename T>
T multiMap(T value, T* _in, T* _out, uint8_t size)
{
    // take care the value is within range
    // value = constrain(value, _in[0], _in[size-1]);
    if (value <= _in[0])
        return _out[0];
    if (value >= _in[size - 1])
        return _out[size - 1];

    // search right interval
    uint8_t pos = 1; // _in[0] already tested
    while (value > _in[pos])
        pos++;

    // this will handle all exact "points" in the _in array
    if (value == _in[pos])
        return _out[pos];

    // interpolate in the right segment for the rest
    return (value - _in[pos - 1]) * (_out[pos] - _out[pos - 1]) / (_in[pos] - _in[pos - 1]) + _out[pos - 1];
}
