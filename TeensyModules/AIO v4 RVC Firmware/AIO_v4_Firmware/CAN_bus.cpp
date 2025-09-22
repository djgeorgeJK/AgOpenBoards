  
   
    

/********************************************************************************
 * INCLUDE DIRECTIVES
 ********************************************************************************/
#include <Arduino.h>
#include <stdint.h>

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
MCP2515 mcp2515(CAN_CS_PIN, 1000000); // CS, SPI speed, MOSI, MISO, SCK
can_frame_t canMsg1 = {
    .can_id = 0x321,
    .can_dlc = 8,
    .data = {0xFF, 0xFF, 0xFF, 0xFF, 0x12, 0xFF, 0xFF, 0xFF}    
};
can_frame_t canMsg2;
can_frame_t canMsgRx;

  
/********************************************************************************
 * PRIVATE FUNCTION PROTOTYPES
 ********************************************************************************/

/********************************************************************************
 * PUBLIC FUNCTION DECLARATIONS
 ********************************************************************************/

void CanBus_Init(void)
{
    Serial.println("CAN Start Init");
    
    //pinMode(CAN_MISO_PIN, INPUT_PULLUP);

    //SPI.setClockDivider(SPI_CLOCK_DIV32);
    // SPI.begin();
    // SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    mcp2515.reset();    
    mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ);    
      

    MCP2515::ERROR err = mcp2515.setNormalMode();
    //MCP2515::ERROR err = mcp2515.setListenOnlyMode();
    Serial.printf("CAN Bus Initialized, err: %d \r\n", err);
}

/// @brief Main CAN Bus Task to read and print any incoming messages
/// @param  
void CanBus_Task(void)
{
    static bool oncePrint = false;
    if (!oncePrint) {
        Serial.printf("CAN Task Started\r\n");
        oncePrint = true;
        canMsgRx.can_id = 0x12345678;
    }
    
    
    
    
    if (Serial.available() > 0) 
    {   byte low = Serial.read();
        if (low == 's')
        {
            mcp2515.sendMessage(&canMsg1);
            Serial.println("CAN message sent \r\n");
        }    


        if (low == '1')
        {
           if (mcp2515.readMessage(&canMsgRx) == MCP2515::ERROR_OK) 
            {
                Serial.printf("%X, dlc %X", canMsgRx.can_id, canMsgRx.can_dlc); // print ID and DLC
                Serial.printf("Zprava prijata ");         
            
                // for (int i = 0; i<canMsgRx.can_dlc; i++)  
                // {  // print the data
                //     Serial.println(canMsgRx.data[i],HEX);
                //     Serial.println(" ");
                // }    
            }
        }    

        if (low == '2')
        {
            uint8_t stat = mcp2515.getRxStatus();
            Serial.printf("Rx Status je: %X\r\n", stat);
        }
    }

}
