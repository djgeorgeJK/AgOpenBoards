#ifndef __BUTTONS_H_GUARD__
#define __BUTTONS_H_GUARD__

typedef enum Remoteswitch // mask to send info to display
{
    RS_WORKING = 0x01,       // H level activate Working mode
    RS_STEERING = 0x02,      // H level activate Steering mode
    RS_REMOTE_SWITCH = 0x04, // zatim nevim
    RS_AUTOSTEER = 0x08      // H level activate Autosteer mode
} Remoteswitch_t;


//Switches
typedef struct Switches
{
  uint8_t remoteSwitch = 0;
  bool workSwitch = false;
  uint8_t steerSwitch = 1;
  uint8_t switchByte = 0;
  uint8_t currentState = 1;
  uint8_t reading;
  uint8_t previous = 0;
}Switches_t;


#endif // __BUTTONS_H_GUARD__