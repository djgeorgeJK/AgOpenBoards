// Single antenna, IMU, & dual antenna code for AgOpenGPS
// If dual right antenna is for position (must enter this location in AgOpen), left Antenna is for heading & roll
//
// connection plan:
// Teensy Serial 7 RX (28) to F9P Position receiver TX1 (Position data)
// Teensy Serial 7 TX (29) to F9P Position receiver RX1 (RTCM data for RTK)
// Teensy Serial 2 RX (7) to F9P Heading receiver TX1 (Relative position from left antenna to right antenna)
// Teensy Serial 2 TX (8) to F9P Heading receiver RX1
// F9P Position receiver TX2 to F9P Heading receiver RX2 (RTCM data for Moving Base)
//
// Configuration of receiver
// Position F9P
// CFG-RATE-MEAS - 100 ms -> 10 Hz
// CFG-UART1-BAUDRATE 460800
// Serial 1 In - RTCM (Correction Data from AOG)
// Serial 1 Out - NMEA GGA
// CFG-UART2-BAUDRATE 460800
// Serial 2 Out - RTCM 1074,1084,1094,1124,1230,4072.0 (Correction data for Heading F9P, Moving Base)
//
// Heading F9P
// CFG-RATE-MEAS - 100 ms -> 10 Hz
// CFG-UART1-BAUDRATE 460800
// Serial 1 Out - UBX-NAV-RELPOSNED
// CFG-UART2-BAUDRATE 460800
// Serial 2 In RTCM

/************************* User Settings *************************/
#define GPS_SIMULATE_GPS_BOARD
//#define GPC_ALLOW_GPS_2

// Serial Ports
#ifdef PLATFORMIO
    #define HwSerial    HardwareSerialIMXRT
#else
    #define HwSerial    HardwareSerial
#endif

#define SerialAOG Serial                    // AgIO USB conection
#define SerialRTK Serial3                   // RTK radio
HardwareSerialIMXRT *SerialGPS = &Serial1;  // Main postion receiver (GGA)
#ifdef GPC_ALLOW_GPS_2
    HardwareSerialIMXRT *SerialGPS2 = &Serial7; // Dual heading receiver
#endif
HardwareSerialIMXRT *SerialIMU = &Serial5;  // IMU BNO-085

// Baud rates
const int32_t baudGPS = 460800;
const int32_t baudRTK = 115200; // most are using Xbee radios with default of 115200

#define ImuWire         Wire // SCL=19:A5 SDA=18:A4
#define RAD_TO_DEG_X_10 572.95779513082320876798154814105

/*****************************************************************/

#include "zNMEAParser.h"
#include <Wire.h>
#include "BNO_RVC.h"
#include "zEthernet.h"
#include "Machine_UDP.h"
#include "gpio.h"
#include "global.h"
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include "CAN_bus.h"

// Roomba Vac mode for BNO085 and data
BNO_rvc rvc = BNO_rvc();
BNO_rvcData BnoData;
bool useBNO08xRVC = false;
bool useMachine = true;

ConfigIP_t networkAddress; // 3 bytes

byte CK_A = 0;
byte CK_B = 0;


// Speed pulse output
elapsedMillis speedPulseUpdateTimer = 0;

// Used to set CPU speed
extern "C" uint32_t set_arm_clock(uint32_t frequency); // required prototype

bool useDual = false;
bool dualReadyGGA = false;
bool dualReadyRelPos = false;

elapsedMillis GGAReadyTime = 10000;
elapsedMillis EthernetCheck_msCounter = 1000;

// Dual
double headingcorr = 900; // 90deg heading correction (90deg*10)

double baseline = 0;
double rollDual = 0;
double relPosD = 0;
double heading = 0;

byte ackPacket[72] = {0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

uint8_t GPSrxbuffer[SERIAL_BUFFER_SIZE];  // Extra serial rx buffer
uint8_t GPStxbuffer[SERIAL_BUFFER_SIZE];  // Extra serial tx buffer
uint8_t GPS2rxbuffer[SERIAL_BUFFER_SIZE]; // Extra serial rx buffer
uint8_t GPS2txbuffer[SERIAL_BUFFER_SIZE]; // Extra serial tx buffer
uint8_t RTKrxbuffer[SERIAL_BUFFER_SIZE];  // Extra serial rx buffer

/* A parser is declared with 3 handlers at most */
NMEAParser<2> parser;

bool isTriggered = false;
bool blink = false;

bool Autosteer_running = true; // Auto set off in autosteer setup

float roll = 0;
float pitch = 0;
float yaw = 0;

// ********************************************************************************************
//                           Function prototypes                                             **
void Read_GPS_FromSerial(void);
void Process_RTK_FromRadio(void);
void Process_RTK_FromUDP(void);

// Setup procedure ------------------------
void setup()
{
    delay(1000);              // Small delay so serial can monitor start up
    set_arm_clock(450000000); // Set CPU speed to 150mhz
    Serial.begin(115200);
    Serial.printf("CPU speed set to: %d\r\n", F_CPU_ACTUAL);
    Serial.printf("Firmware version %d.%d.%d Debug \r\n", FW_MAJ, FW_MIN, FW_PATCH);

    // pinMode(GGAReceivedLED,         OUTPUT);
    pinMode(Power_on_LED, OUTPUT);
    pinMode(Ethernet_Active_LED, OUTPUT);
    pinMode(GPSRED_LED, OUTPUT);
    // pinMode(GPSGREEN_LED,           OUTPUT);
    pinMode(AUTOSTEER_STANDBY_LED, OUTPUT);
    pinMode(AUTOSTEER_ACTIVE_LED, OUTPUT);

    // the dash means wildcard
    parser.setErrorHandler(errorHandler);
    parser.addHandler("G-GGA", GGA_Handler);
    parser.addHandler("G-VTG", VTG_Handler);

    delay(10);
    Serial.println("Start setup");

    SerialGPS->begin(baudGPS);
    SerialGPS->addMemoryForRead(GPSrxbuffer, SERIAL_BUFFER_SIZE);
    SerialGPS->addMemoryForWrite(GPStxbuffer, SERIAL_BUFFER_SIZE);

    delay(10);
    SerialRTK.begin(baudRTK);
    SerialRTK.addMemoryForRead(RTKrxbuffer, SERIAL_BUFFER_SIZE);

    delay(10);
#ifdef GPC_ALLOW_GPS_2
    SerialGPS2->begin(baudGPS);
    SerialGPS2->addMemoryForRead(GPS2rxbuffer, SERIAL_BUFFER_SIZE);
    SerialGPS2->addMemoryForWrite(GPS2txbuffer, SERIAL_BUFFER_SIZE);
#endif

    Serial.println("SerialAOG, SerialRTK, SerialGPS and SerialGPS2 initialized");

    Serial.println("\r\nStarting AutoSteer...");
    autosteerSetup();

    Serial.println("\r\nStarting Ethernet...");
    CanBus_Init();
    EthernetStart();

    SerialIMU->begin(115200);
    rvc.begin(SerialIMU);

    static elapsedMillis rvcBnoTimer = 0;
    Serial.println("\r\nChecking for serial BNO08x");
    BNO_rvcData bnoInitData;
    while (rvcBnoTimer < 1000)
    {
        // check if new Data
        if (rvc.read(&bnoInitData))
        {
            useBNO08xRVC = true;
            Serial.printf("Serial BNO08x Good To Go. X:%d, Y:%d, Z:%d\r\n", bnoInitData.pitchX10, bnoInitData.rollX10, bnoInitData.yawX10);
            imuHandler(bnoInitData);
            break;
        }
    }
    if (!useBNO08xRVC)
        Serial.println("No Serial BNO08x not Connected or Found");

    Serial.println("\r\nEnd setup, waiting for GPS...\r\n");
    Autosteer_running = true;

    int val = analogRead(AN_POT_MY);
    Serial.printf("Wheel AD data: %d\r\n", val);

    Machine_Init();
}

/// @brief Main loop of project
void loop()
{
    Read_GPS_FromSerial();
    Process_RTK_FromRadio();
    Process_RTK_FromUDP();

    // If both dual messages are ready, send to AgOpen
    if (dualReadyGGA == true && dualReadyRelPos == true)
    {
        BuildNmea();
        dualReadyGGA = false;
        dualReadyRelPos = false;
    }

    Read_GPS_2_FromSerial();

    ReceiveUdp();

    if (Autosteer_running)
    {
        autosteerLoop();
        gps_speed();
    }



    // GGA timeout, turn off GPS LED's etc
    if (GGAReadyTime > 10000) // GGA age over 10sec
    {
        // digitalWrite(GPSRED_LED, LOW);
        // digitalWrite(GPSGREEN_LED, LOW);
        useDual = false;
    }

    EthernetTask();
    TaskScheduler();

} // End Loop

//***************************************************************************
//                          Private Functions                               *
//***************************************************************************
void Read_GPS_FromSerial(void)
{ // Read incoming nmea from GPS
    if (SerialGPS->available())
    {
        static bool printed = false;
        parser << SerialGPS->read();
        if (!printed)
        {
            Serial.println("GPS connected !!\r\n");
            printed = true;
        }
    }
}

void Read_GPS_2_FromSerial(void)
{
    static int relposnedByteCount = 0;
    // If anything comes in SerialGPS2 RelPos data
#ifdef GPC_ALLOW_GPS_2
    if (SerialGPS2->available())
    {
        uint8_t incoming_char = SerialGPS2->read(); // Read RELPOSNED from F9P

        // Just increase the byte counter for the first 3 bytes
        if (relposnedByteCount < 4 && incoming_char == ackPacket[relposnedByteCount])
        {
            relposnedByteCount++;
        }
        else if (relposnedByteCount > 3)
        {
            // Real data, put the received bytes in the buffer
            ackPacket[relposnedByteCount] = incoming_char;
            relposnedByteCount++;
        }
        else
        {
            // Reset the counter, becaues the start sequence was broken
            relposnedByteCount = 0;
        }
    }
#endif
    // Check the message when the buffer is full
    if (relposnedByteCount > 71)
    {
        if (calcChecksum())
        {
            // if(deBug) Serial.println("RelPos Message Recived");
            digitalWrite(GPSRED_LED, LOW); // Turn red GPS LED OFF (we are now in dual mode so green LED)
            useDual = true;
            relPosDecode(BnoData);
        }
        relposnedByteCount = 0;
    }
}

void Process_RTK_FromRadio(void)
{ // Check for RTK via Radio
    if (SerialRTK.available())
    {
        SerialGPS->write(SerialRTK.read());
        Serial.println(" Reading SerialRTK UART\r\n");
    }
}

void TaskScheduler(void)
{
    static uint32_t scheduler_last_cntr;
    static BNO_rvcData bnoData;

    if (scheduler_last_cntr != systick_millis_count)
    {
        if ((systick_millis_count % 1000) == 0) // each second
        {
            // int val = analogRead(AN_POT_MY);
            // Serial.printf("analog A10 is: %d\r\n", val);
            // Serial.printf("1 sec print\r\n");
        }
        scheduler_last_cntr = systick_millis_count;

        if ((systick_millis_count % 2) == 0) // each 2 ms
        {
            // Buttons_Sample();
        }

        if ((systick_millis_count % MACHINE_LOOP_PERIOD_MS) == 0) // each 100 ms
        {
            Machine_loop();
            CanBus_Task();

            // RVC BNO08x
            if (rvc.read(&bnoData))
             {
                useBNO08xRVC = true;
            }else
            {
                Serial.printf("Serial BNO08x not responding\r\n");
                bnoData = { 0 };
            }

            if (useBNO08xRVC  )
            {
                imuHandler(bnoData); // Get IMU data ready
            }
            BnoData = bnoData;
        }

        if ((systick_millis_count % 1000) == 0) // each 1000 msec
        {
            //Serial.printf("Serial BNO08x:X:%d, Y:%d, Z:%d\r\n", bnoData.pitchX10, bnoData.rollX10, bnoData.yawX10);
        }

    }
}

bool calcChecksum()
{
    CK_A = 0;
    CK_B = 0;

    for (int i = 2; i < 70; i++)
    {
        CK_A = CK_A + ackPacket[i];
        CK_B = CK_B + CK_A;
    }

    return (CK_A == ackPacket[70] && CK_B == ackPacket[71]);
}
