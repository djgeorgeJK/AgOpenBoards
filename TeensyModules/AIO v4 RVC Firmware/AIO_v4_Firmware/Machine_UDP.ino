    //Machine Control - Brian Tee - Cut and paste from everywhere


    //-----------------------------------------------------------------------------------------------
    // Change this number to reset and reload default parameters To EEPROM
    #define EEP_Ident 0x5425  
    
   
    //-----------------------------------------------------------------------------------------------

    #include <EEPROM.h> 
    #include <Wire.h>
    #include "Arduino.h"
        
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

    //Program counter reset
    void(*resetFunc) (void) = 0;


    /*
    * Functions as below assigned to pins
    0: -
    1 thru 16: Section 1,Section 2,Section 3,Section 4,Section 5,Section 6,Section 7,Section 8,
                Section 9, Section 10, Section 11, Section 12, Section 13, Section 14, Section 15, Section 16,
    17,18    Hyd Up, Hyd Down,
    19 Tramline,
    20: Geo Stop
    21,22,23 - unused so far*/    
    uint8_t pin[] = { 1,2,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

    //read value from Machine data and set 1 or zero according to list
    uint8_t relayState[] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

    //hello from AgIO
    //uint8_t helloFromMachine[] = { 128, 129, 123, 123, 5, 0, 0, 0, 0, 0, 71 };

   
   
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

    void Machine_loop()
    {
       
    }

    void Machine_ProcessData(uint8_t * udpdata)
    {
        for (int i = 0; i < 10; i++)
        {
            SerialUSB.printf("Dat %d", udpdata[i]);
        }
        SerialUSB.printf("\r\n");
    }
    void SetRelays(void)
    {
              //GeoStop
      

        if (pin[0]) digitalWrite(4, relayState[pin[0] - 1]);
        if (pin[1]) digitalWrite(5, relayState[pin[1] - 1]);
        if (pin[2]) digitalWrite(6, relayState[pin[2] - 1]);
        if (pin[3]) digitalWrite(7, relayState[pin[3] - 1]);

        if (pin[4]) digitalWrite(8, relayState[pin[4] - 1]);
        if (pin[5]) digitalWrite(9, relayState[pin[5] - 1]);

        //if (pin[6]) digitalWrite(10, relayState[pin[6]-1]);
        //if (pin[7]) digitalWrite(11, relayState[pin[7]-1]);

        //if (pin[8]) digitalWrite(12, relayState[pin[8]-1]);
        //if (pin[9]) digitalWrite(4, relayState[pin[9]-1]);

        //if (pin[10]) digitalWrite(IO#Here, relayState[pin[10]-1]);
        //if (pin[11]) digitalWrite(IO#Here, relayState[pin[11]-1]);
        //if (pin[12]) digitalWrite(IO#Here, relayState[pin[12]-1]);
        //if (pin[13]) digitalWrite(IO#Here, relayState[pin[13]-1]);
        //if (pin[14]) digitalWrite(IO#Here, relayState[pin[14]-1]);
        //if (pin[15]) digitalWrite(IO#Here, relayState[pin[15]-1]);
        //if (pin[16]) digitalWrite(IO#Here, relayState[pin[16]-1]);
        //if (pin[17]) digitalWrite(IO#Here, relayState[pin[17]-1]);
        //if (pin[18]) digitalWrite(IO#Here, relayState[pin[18]-1]);
        //if (pin[19]) digitalWrite(IO#Here, relayState[pin[19]-1]);
    }
