#ifndef __GLOBAL_GUARD__
#define __GLOBAL_GUARD__

#define BOARD_01_2026_v1_3
// jinak je rucne pajena deska


//Status LED's - used RED LED on Teensy BOARD
#define STARTUP_LED             13      //Teensy onboard LED - used only for startup
#define GGAReceivedLED          13      //Teensy onboard LED - used now for SCK-CAN, Active High
#define MY_LED_1                32      // JiKa board LED1

/* Ethernet cable Green LED is driven by ETHpy */
#define Ethernet_Active_LED     6       //Green

/* unused LED */
#define GPSRED_LED              3       //Red (Flashing = NO IMU or Dual, ON = GPS fix with IMU)
#define GPSGREEN_LED            50      //Green (Flashing = Dual bad, ON = Dual good)
#define AUTOSTEER_STANDBY_LED   50      //Red
#define AUTOSTEER_ACTIVE_LED    50      //Green


#define AN_POT_MY               A10     //24

// Velocity (MPH speed) PWM pin
#define GP_VELOCITY_PIN         28

//--------------------------- sections for spreader
#if defined (BOARD_01_2026_v1_3)
    #define GP_RE1                  33
    #define GP_RE2                  37
    #define GP_RE3                  36
    #define GP_RE4                  35
    #define GP_RE5                  34
#else // rucne pajeny desky
    #define GP_RE1                  33
    #define GP_RE2                  36
    #define GP_RE3                  37
    #define GP_RE4                  14
#endif


/* Motor outputs*/
// Dir1 for Cytron Dir, Both L and R enable for IBT2
#define MC_DIRECTION_PIN            30
#define MC_PWM_1                    29      //PWM for both Cytron and IBT2

//--------------------------- Switch Input Pins ------------------------
#define STEERSW_PIN     32
#define REMOTE_PIN      26
#define DEBUG_PIN       41
#if defined (BOARD_01_2026_v1_3)
    #define WORKSW_PIN      27
#else // rucne pajeny desky
    #define WORKSW_PIN      31
#endif

//Define sensor pin for current or pressure sensor
#define CURRENT_SENSOR_PIN    A17
#define PRESSURE_SENSOR_PIN   A11

/* SPI for CAN BUS */
#define CAN_MOSI_PIN    11
#define CAN_MISO_PIN    12
#define CAN_SCK_PIN     13
#define CAN_CS_PIN      10

#define CAN_INT2      15
#define CAN_INT1      14


/* Serial ports */

/* BNO uses Serial 5  pin 21 and 20 */

#endif //__GLOBAL_GUARD__