
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
 

