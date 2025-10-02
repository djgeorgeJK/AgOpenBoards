  
   
    

/********************************************************************************
 * INCLUDE DIRECTIVES
 ********************************************************************************/
#include <Arduino.h>
#include <stdint.h>
#include "can_bus.h"


#include "global.h"
#include "modules\mcp2515\mcp2515.h"
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

#define CAN_TASK_PERIOD_MS      100u
#define CAN_SEED_TIMEOUT_MS     2000u

typedef struct SeedingState
{
    bool leftSideActive;
    bool rightSideActive;
    uint16_t seedMessageTimeout;
}SeedingState_t;
/********************************************************************************
 * VARIABLE DECLARATIONS
 ********************************************************************************/
MCP2515 mcp2515(CAN_CS_PIN, 1000000); // CS, SPI speed, MOSI, MISO, SCK
can_frame_t canMsg1 = {
    .can_id = 0x0C300840 | CAN_EFF_FLAG,
    .can_dlc = 2,
    //.data = {0xFF, 0xFF, 0xFF, 0xFF, 0x12, 0xFF, 0xFF, 0xFF}    
    .data = {0x80, 0x12}    
};
can_frame_t canMsg2;
can_frame_t canMsgRx;

static SeedingState_t seedingState = 
{
    .leftSideActive = false, 
    .rightSideActive = false,
    .seedMessageTimeout = CAN_SEED_TIMEOUT_MS
};
  
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
      

    

    /* Chci cist zpravy:
     *  - 0x1ce68226   A8 29  nebo 2A je aktivni vysev leve , prave strany
     * Mask -   1 znamena ze bit se bude testovat filtrem
     * Filter pak muzi videt i 0 i 1
     * U externded message koukaji na spodnich 29 bitu(0x1FFFFFFF)
     * S tim to filtrem uz klasickej read vrac jen tuto zpravu
     */
    mcp2515.setFilterMask(MCP2515::MASK0, true, 0x1FFFFFFF);
    mcp2515.setFilter(MCP2515::RXF0, true, 0x1ce68226); // RXB0 - Extended

    MCP2515::ERROR err = mcp2515.setNormalMode();
    //MCP2515::ERROR err = mcp2515.setListenOnlyMode();
    //MCP2515::ERROR err = mcp2515.setLoopbackMode();
    Serial.printf("CAN Bus Initialized, err: %d \r\n", err);
    seedingState.seedMessageTimeout = CAN_SEED_TIMEOUT_MS;
}

//#include <WDT_T4.h>
//Watchdog_t4 wd;
/// @brief Main CAN Bus Task to read and print any incoming messages
/// @param  
void CanBus_Task(void)
{
    static bool oncePrint = false;
    if (!oncePrint) {
        Serial.printf("CAN Task Started\r\n");
        oncePrint = true;
    }

    if(seedingState.seedMessageTimeout > CAN_TASK_PERIOD_MS)
    {
        seedingState.seedMessageTimeout -= CAN_TASK_PERIOD_MS;
        //Serial.printf("%d\r\n", seedingState.seedMessageTimeout);
    }
    else    
    {
        seedingState.leftSideActive = false;
        seedingState.rightSideActive = false;        
        seedingState.seedMessageTimeout = CAN_SEED_TIMEOUT_MS;
        Serial.printf("Mazu left right priznaky");
    }
    
    if (mcp2515.readMessage(&canMsgRx) == MCP2515::ERROR_OK) 
    {
        canMsgRx.can_id &= CAN_EFF_MASK; // mask off the EFF/RTR/ERR flags
        Serial.printf("Id %X, dlc %X", canMsgRx.can_id, canMsgRx.can_dlc); // print ID and DLC
        //Serial.printf("Zprava prijata ");         
    
        for (int i = 0; i<canMsgRx.can_dlc; i++)  
        {  // print the data
                Serial.printf("0x%X ",canMsgRx.data[i]);                 
        }    
        Serial.println("");


        // Can zprava xx68226 00 02 prijde kdyz se zmackne tlacitko sej na miste 1D 
        if(canMsgRx.data[0] == 0x00 && canMsgRx.data[1] == 0x02)
        {
            Serial.printf("Zmackle tlacitko\r\n");     
            seedingState.rightSideActive = true;      
            seedingState.seedMessageTimeout = CAN_SEED_TIMEOUT_MS; 
        }
        
        if(canMsgRx.data[0] == 0xA8)    // zprava na aktivni sekce
        {
            if(canMsgRx.data[1] == 0x2A)
            {
                // right side active
                seedingState.rightSideActive = true;
                Serial.printf("Leva ON"); 
                seedingState.seedMessageTimeout = CAN_SEED_TIMEOUT_MS;
            }
            else if (canMsgRx.data[1] == 0x29)
            {
                // left side active
                seedingState.leftSideActive = true;
                Serial.printf("Prava ON"); 
                seedingState.seedMessageTimeout = CAN_SEED_TIMEOUT_MS;
            }
        }
    }
    
    
    /*  Processing CLI */
    if (Serial.available() > 0) 
    {
        char charIn = Serial.read();    
        if (charIn == 's')
        {
            mcp2515.sendMessage(&canMsg1);
            Serial.println("CAN message sent \r\n");
            canMsg1.data[0] ++;
        }    


        if (charIn == '1')
        {
            uint8_t stat = mcp2515.getStatus();
            if(stat != 0)
            { 
                Serial.printf("Status je: %X\r\n", stat);
            }          
            
        }    

        if (charIn == '2')
        {
            uint8_t stat = mcp2515.getRxStatus();
            Serial.printf("Rx Status je: %X\r\n", stat);
        }
        

        if (charIn == 'r')
        {
            Serial.printf("Rebooting...."); 
            delay(800);
            //wdt_disable();
            //wdt_enable(WDTO_15MS);
            
            
        
            //SCB_AIRCR = 0x05FA0004;  // Request system reset
            _reboot_Teensyduino_();
        }
    }

}

bool CanBus_IsLeftSideActive(void)
{
    return seedingState.leftSideActive;
}   

bool CanBus_IsRightSideActive(void)
{
    return seedingState.rightSideActive;
}  

bool CanBus_IsSeedingActive(void)
{
    return (seedingState.rightSideActive) || (seedingState.leftSideActive);
}  