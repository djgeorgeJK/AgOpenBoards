    //Machine Control - Brian Tee - Cut and paste from everywhere




    //-----------------------------------------------------------------------------------------------

/********************************************************************************
 * INCLUDE DIRECTIVES
 ********************************************************************************/
#include <stdint.h>
#include "Machine_UDP.h"
#include <EEPROM.h>
#include <Wire.h>
#include "Arduino.h"
#include "global.h"
#include "gpio.h"
#include "Buttons.h"

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

};  Config aogConfig;   //8 bytes


/* Status of hydraulic,tramlines and Sections  */
bool TramLineL, TramLineR;
uint16_t SectionState;      // each bit means one of 16 section
hydLiftState_t HydraulicLift;

/* Functions as below assigned to pins */
// define pins to available for Section controls listed in GUI as Pin 1 .. Pin24 which is index 0..23
const uint8_t hwPinAsignment [] = {GP_RE1,GP_RE2,GP_RE3,GP_RE4};   //so pin1 will be tennsy
const uint8_t hwPinTramAsignment [] = {40,41};


/* This aray holds definition of Which virtual Pin 1..24 holds which functionlaity.
*  this array is re-configurate over GUI
*  0: - not selected
*  1..16:  Section 1,Section 2,Section 3,Section 4,Section 5,Section 6,Section 7,Section 8,
*          Section 9, Section 10, Section 11, Section 12, Section 13, Section 14, Section 15, Section 16,
*  17,18   Hyd Up, Hyd Down,
*  19      Tramline,
*  20: Geo Stop
*  21,22,23 - unused so far
*    // index is virtual pin number
*    // value as descripbed above - 1-16,17,18,22-23 - is purpose of use */
uint8_t PinToSection[24] = { 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,\
                             17    };


/********************************************************************************
 * PRIVATE FUNCTION PROTOTYPES
 ********************************************************************************/
  //Program counter reset
    void(*resetFunc) (void) = 0;
    void Machine_ProcessRelays(void);
/********************************************************************************
 * PUBLIC FUNCTION DECLARATIONS
 ********************************************************************************/

void Machine_Init(void)
{
    for(uint i = 0; i< sizeof(hwPinAsignment); ++i)
    {
        pinMode(hwPinAsignment[i], OUTPUT);
    }
    pinMode(hwPinTramAsignment[0], OUTPUT);
    pinMode(hwPinTramAsignment[1], OUTPUT);
    uint16_t ee_ready;
    EEPROM.get(EE_ADDR_SECTI_HEAD, ee_ready);              // read identifier

    if (ee_ready == EEP_Ident)
    {
        EEPROM.get(EE_ADDR_SECTI, PinToSection);
        EEPROM.get(EE_ADDR_AOGCFG, aogConfig);
        Serial.printf(" Machine EEPROM OK \r\n");
    }
}

void Machine_setup()
{
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


/// @brief Call witn 50ms   #define MACHINE_LOOP_PERIOD_MS      100
void Machine_loop(void)
{

    Machine_ProcessRelays();
}

void Machine_ProcessData(uint8_t * udpData)     // 239 Machine data, goes from AOG every 10ms
{

    //uTurn = udpData[5];
    //uint8_t localGpsSpeed = (float)udpData[6];//actual speed times 10

    uint8_t hydLift = udpData[7];   // when change, value 1 - start Lower, value 2 - rising
    uint8_t tramline = udpData[8];  //bit 0 is right bit 1 is left

    // From AIO goes 16 bit status of sections
    uint16_t sectionStates = ((uint16_t)udpData[12] << 8 ) | (uint16_t)udpData[11];          // read relay control from AgOpenGPS sc1to8 = 11;sc9to16 = 12;

    // SerialUSB.printf("Data 239 mchine");
    // for (uint8_t i=7;i < 25; i++)
    // {
    //     Serial.printf("%3d, ", udpData[i]);
    // }

    // SerialUSB.printf("\r\n");


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

/* Trigger is in APP interface by machine module */
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
    EEPROM.put(EE_ADDR_AOGCFG, aogConfig);
    Serial.printf("Process AOG config, U1:%d; U2:%d; U3:%d; U4:%d \r\n", aogConfig.user1,aogConfig.user2,aogConfig.user3,aogConfig.user4 );
}

void Machine_ProcessRelayConfig(uint8_t * udpData)  // 236 machine Relay Pin Settings
{
    //assignment pin to section number. E.g in PinToSection[1] will be # 2 , section no 2
    // where then hast to be this PinToSection[1] assigned to some HW pin
    for (uint8_t i = 0; i < 24; i++)
    {
        PinToSection[i] = udpData[i + 5];        // 5 is first data byte
        Serial.printf("v Pin %d, na je sekce %d\r\n", i, PinToSection[i]);
    }

    //save in EEPROM and restart
    EEPROM.put(EE_ADDR_SECTI_HEAD, EEP_Ident);
    EEPROM.put(EE_ADDR_SECTI, PinToSection);
}

extern Switches_t ButtState;
/* HW write of relays*/
void Machine_ProcessRelays(void)
{
    static uint16_t sectionStateLast;
    bool printChange = false;
    // we need to assign variable of virtual pin to proper pin.
    // where hwPinAsignment [] = {33,36,37,13};   //are reap used pin #1 .. 24
    // when value is 0, like not assigned

    //if (PinToSection[0]) digitalWrite(hwPinAsignment[0], SectionState & (1 << (PinToSection[0] - 1)));    // do pinu 4 zapis hodnotu co prisla v Cfg Pinu 1

    if (sectionStateLast != SectionState)
    {
        sectionStateLast = SectionState;
        Serial.printf("Sekce-Novy stav %d \r\n", SectionState);
        printChange = true;
    }


    for (uint8_t pin = 0; pin < sizeof(hwPinAsignment); pin++)      // max honota muze by i 24, ale nema smysl, ze mam jen 4 piny
    {
        uint8_t value = SectionState & (1 << (PinToSection[pin] - 1));
        if( PinToSection[pin] <= 16 )       // 16 sections, 0 not used
        {
            if ((aogConfig.user1 & UN1_WS_DIS_SEC) && (ButtState.workSwitch == 1))   // when is this feature allowed
            { // not change section pins
            }
            else
            {
                digitalWrite(hwPinAsignment[pin], value );
                if (printChange)
                {
                //Serial.printf("Zapisu na pin %d, hodnotu %d \r\n", hwPinAsignment[pin], value ? 0 : 1);
                }
            }
        }else
        {
            Serial.printf("Sekce-Spatna sekce %d\r\n", PinToSection[pin]);
        }
    }



    //digitalWrite(9, TramLineL);
    //digitalWrite(10, TramLineR);

}
