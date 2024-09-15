    //Machine Control - Brian Tee - Cut and paste from everywhere


    //-----------------------------------------------------------------------------------------------
    // Change this number to reset and reload default parameters To EEPROM
    //#define EEP_Ident 0x5425  
    
   
    //-----------------------------------------------------------------------------------------------

/********************************************************************************
 * INCLUDE DIRECTIVES
 ********************************************************************************/
#include <EEPROM.h> 
#include <Wire.h>
#include "Arduino.h"

/********************************************************************************
 * DEFINITIONS, ENUMS, STRUCTURES AND TYPEDEFS
 ********************************************************************************/     

typedef struct HydLiftState
{
    uint8_t LiftLastState;
    uint8_t timerRising;
    uint8_t timerLovering;
}hydLiftState_t;
/********************************************************************************
 * VARIABLE DECLARATIONS
 ********************************************************************************/
//Variables for config - 0 is false  
struct Config {
    uint8_t raiseTime = 2;
    uint8_t lowerTime = 4;
    uint8_t enableToolLift = 0;
    uint8_t isRelayActiveHigh = 0; //if zero, active low (default)

    uint8_t user1 = 0; //user defined values set in machine tab
    uint8_t user2 = 0;
    uint8_t user3 = 0;
    uint8_t user4 = 0;

};  Config aogConfig;   //4 bytes


/* Status of hydraulic,tramlines and Sections  */
bool TramLineL, TramLineR;
uint16_t SectionState;      // each bit means one of 16 section
hydLiftState_t HydraulicLift;

// define pins to available forSection controls listed in app as Pin 1 .. pin24 which is index 0..23
const uint8_t hwPinAsignment [] = {34,35,36,37};   //so pin1 will be tennsy hwio7
const uint8_t hwPinTramAsignment [] = {40,41};

/*
* Functions as below assigned to pins
*/   

/* This aray holds definition of Which virtual Pin 1..24 holds which functionlaity
*  0: - not selected
*  1..16:  Section 1,Section 2,Section 3,Section 4,Section 5,Section 6,Section 7,Section 8,
*          Section 9, Section 10, Section 11, Section 12, Section 13, Section 14, Section 15, Section 16,
*  17,18   Hyd Up, Hyd Down,
*  19      Tramline,
*  20: Geo Stop
*  21,22,23 - unused so far*/
uint8_t PinToSection[24] = { 1,2,3,0 };


/********************************************************************************
 * PRIVATE FUNCTION PROTOTYPES
 ********************************************************************************/
  //Program counter reset
    void(*resetFunc) (void) = 0;
/********************************************************************************
 * PUBLIC FUNCTION DECLARATIONS
 ********************************************************************************/

void Machine_Init()
{
    for(uint i = 0; i< sizeof(hwPinAsignment); ++i)
    {
        pinMode(hwPinAsignment[i], OUTPUT);
    }
    pinMode(hwPinTramAsignment[0], OUTPUT);
    pinMode(hwPinTramAsignment[1], OUTPUT);
    uint16_t a;
    EEPROM.get(0, a);              // read identifier

}    
    void Machine_setup()
    {
        
        // EEPROM.get(0, EEread);              // read identifier

        // if (EEread != EEP_Ident)   // check on first start and write EEPROM
        // {
        //     EEPROM.put(0, EEP_Ident);
        //     EEPROM.put(6, aogConfig);
        //     EEPROM.put(20, pin);
        //     EEPROM.put(50, networkAddress);
        // }
        // else
        // {
        //     EEPROM.get(6, aogConfig);
        //     EEPROM.get(20, pin);
        //     EEPROM.get(50, networkAddress);
        // }


        //register to port 8888
        

        //set the pins to be outputs (pin numbers)
        pinMode(4, OUTPUT);
        pinMode(5, OUTPUT);
        pinMode(6, OUTPUT);
        pinMode(7, OUTPUT);
        pinMode(8, OUTPUT);
        pinMode(9, OUTPUT);

        //SerialUSB.println("Setup of Machine complete");

    }

    #define MACHINE_LOOP_PERIOD_MS      50
    /// @brief Call witn 50ms
    void Machine_loop()
    {
       
    }

    void Machine_ProcessData(uint8_t * udpData)     // 239 Machine data, goes from AOG every 10ms 
    {        
        
        //uTurn = udpData[5];
        uint8_t localGpsSpeed = (float)udpData[6];//actual speed times 10

        uint8_t hydLift = udpData[7];   // when change, value 1 - start Lower, value 2 - rising
        uint8_t tramline = udpData[8];  //bit 0 is right bit 1 is left

        // From AIO goes 16 bit status of sections
        uint16_t sectionStates = ((uint16_t)udpData[12] << 8 ) | (uint16_t)udpData[11];          // read relay control from AgOpenGPS sc1to8 = 11;sc9to16 = 12;
        
        SerialUSB.printf("Data 239 mchine");
        for (uint8_t i=7;i < 25; i++)
        {
            Serial.printf("%3d, ", udpData[i]);
        }
        
        SerialUSB.printf("\r\n");


        if (aogConfig.isRelayActiveHigh)
        {
            tramline = 255 - tramline;
            sectionStates = ~sectionStates;
        }

        // Fill global variables
        TramLineR = tramline & 0x01;
        TramLineL = tramline & 0x02;
        SectionState = sectionStates;

        if ( hydLift != HydraulicLift.LiftLastState)
        {
            HydraulicLift.LiftLastState = hydLift;    
            switch (hydLift)
            {                
            case 1: //lower
                HydraulicLift.timerLovering = aogConfig.lowerTime * 5;
                break;                
            case 2: //raise
                HydraulicLift.timerRising = aogConfig.raiseTime * 5;
                break;
            default: 
            break;
            }
        }
    }

    void Machine_ProcessConfig(uint8_t * udpData)   // PGN - 238 - EE - goes when Machine module setup- 
    {
        aogConfig.raiseTime = udpData[5];
        aogConfig.lowerTime = udpData[6];
        aogConfig.enableToolLift = udpData[7];
        //set1 
        uint8_t sett = udpData[8];  //setting0     
        if (bitRead(sett, 0)) aogConfig.isRelayActiveHigh = 1; else aogConfig.isRelayActiveHigh = 0;
        
        aogConfig.user1 = udpData[9];   // in AOG are 4 user values to send heree.g some specific config of teensy
        aogConfig.user2 = udpData[10];
        aogConfig.user3 = udpData[11];
        aogConfig.user4 = udpData[12];

        //save in EEPROM and restart
        EEPROM.put(6, aogConfig);
    }

    void Machine_ProcessRelayConfig(uint8_t * udpData)  // 236 machine Relay Pin Settings 
    {            
        //assignment pin to section number. E.g in PinToSection[1] will be # 9 li (th section)
        for (uint8_t i = 0; i < 24; i++)
        {
            PinToSection[i] = udpData[i + 5];        // 5 is first data byte
            Serial.printf("REle %d\r\n", PinToSection[i]);
        }

        //save in EEPROM and restart
        EEPROM.put(20, PinToSection);
    }
    
    
    /* HW write of relays*/
    void Machine_ProcessRelays(void)
    {          
        // we need to assign variable of relay state to proper pin.
        // when value is 0, like not assigned

        if (PinToSection[0]) digitalWrite(hwPinAsignment[0], SectionState & (1 << (PinToSection[0] - 1)));    // do pinu 4 zapis hodnotu co prisla v Cfg Pinu 1
        
        for (uint i = 0; i < sizeof(hwPinAsignment); ++i)
        {
            if (PinToSection[i]) digitalWrite(hwPinAsignment[i], SectionState & (1 << (PinToSection[i] - 1)));
        }
        digitalWrite(9, TramLineL);
        digitalWrite(10, TramLineR);
    }
