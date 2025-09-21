#ifndef __GLOBAL_GUARD__
#define __GLOBAL_GUARD__

//Status LED's
#define GGAReceivedLED          13      //Teensy onboard LED
#define Power_on_LED            5       //Red
#define Ethernet_Active_LED     6       //Green
#define GPSRED_LED              9       //Red (Flashing = NO IMU or Dual, ON = GPS fix with IMU)
#define GPSGREEN_LED            50      //Green (Flashing = Dual bad, ON = Dual good)
#define AUTOSTEER_STANDBY_LED   50      //Red
#define AUTOSTEER_ACTIVE_LED    50      //Green
#define AN_POT_MY               A10     //24

// Velocity (MPH speed) PWM pin
#define GP_VELOCITY_PIN         29

//--------------------------- sections for spreader
#define GP_RE1                  33
#define GP_RE2                  36
#define GP_RE3                  37
#define GP_RE4                  14

//--------------------------- Switch Input Pins ------------------------
#define STEERSW_PIN     32
#define WORKSW_PIN      31    
#define REMOTE_PIN      30
#define DEBUG_PIN       41

//Define sensor pin for current or pressure sensor
#define CURRENT_SENSOR_PIN    A17
#define PRESSURE_SENSOR_PIN   A11

/* SPI for CAN BUS */
#define CAN_MOSI_PIN    11
#define CAN_MISO_PIN    12
#define CAN_SCK_PIN     13
#define CAN_CS_PIN      10


#endif //__GLOBAL_GUARD__