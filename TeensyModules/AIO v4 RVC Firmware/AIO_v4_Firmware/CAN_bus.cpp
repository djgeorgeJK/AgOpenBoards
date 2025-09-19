  
   
    

/********************************************************************************
 * INCLUDE DIRECTIVES
 ********************************************************************************/
#include <stdint.h>

#include "Arduino.h"
#include "global.h"
#include "mcp2515.h"
#include "gpio.h"

/********************************************************************************
 * DEFINITIONS, ENUMS, STRUCTURES AND TYPEDEFS
 ********************************************************************************/     
//  !! Set Brand via Service Tool (Serial Monitor) !! 
//  0 = Claas (1E/30 Navagation Controller, 13/19 Steering Controller) - See Claas Notes on Service Tool Page
//  1 = Valtra, Massey Fergerson (Standard Danfoss ISO 1C/28 Navagation Controller, 13/19 Steering Controller) Mccormick
//  2 = CaseIH, New Holland (AA/170 Navagation Controller, 08/08 Steering Controller)
//  3 = Fendt (2C/44 Navagation Controller, F0/240 Steering Controller)
//  4 = JCB (AB/171 Navagation Controller, 13/19 Steering Controller)
//  5 = FendtOne - Same as Fendt but 500kbs K-Bus.
//  6 = Lindner (F0/240 Navagation Controller, 13/19 Steering Controller)
//  7 = AgOpenGPS - Remote CAN/PWM module (1C/28 Navagation Controller, 13/19 Steering Controller)

/********************************************************************************
 * VARIABLE DECLARATIONS
 ********************************************************************************/

 #include <FlexCAN_T4.h>
//FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_256> K_Bus;    //Tractor / Control Bus
//FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_256> ISO_Bus;  //ISO Bus for kverneland


can_frame_t canMsg1;
can_frame_t canMsg2;
can_frame_t canMsgRx;


MCP2515 mcp2515(CAN_CS_PIN, 400000); // CS, SPI speed, MOSI, MISO, SCK
/********************************************************************************
 * PRIVATE FUNCTION PROTOTYPES
 ********************************************************************************/

/********************************************************************************
 * PUBLIC FUNCTION DECLARATIONS
 ********************************************************************************/

void CanBus_Init(void)
{
    // ISO_Bus.begin();
    // ISO_Bus.setBaudRate(250000);
    // ISO_Bus.enableFIFO();
    // ISO_Bus.setFIFOFilter(REJECT_ALL);


    mcp2515.reset();
    mcp2515.setBitrate(CAN_125KBPS);
    //mcp2515.setNormalMode();
    mcp2515.setListenOnlyMode();
}


void CanBus_Task(void)
{
    Serial.print(" Jsem v SPI tasku\r\n"); 
  if (mcp2515.readMessage(&canMsgRx) == MCP2515::ERROR_OK) {
    Serial.print(canMsgRx.can_id, HEX); // print ID
    Serial.print(" "); 
    Serial.print(canMsgRx.can_dlc, HEX); // print DLC
    Serial.print(" ");
    
    for (int i = 0; i<canMsgRx.can_dlc; i++)  {  // print the data
      Serial.print(canMsgRx.data[i],HEX);
      Serial.print(" ");
    }

    Serial.println();      
  }
}