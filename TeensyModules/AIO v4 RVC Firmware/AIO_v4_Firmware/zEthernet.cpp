#include <stdint.h>
#include "zEthernet.h"
// #include <IPAddress.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include "gpio.h"

// IP & MAC address of this module of this module
uint8_t Eth_myip[4] = {0, 0, 0, 0}; // This is now set via AgIO
uint8_t mac[] = {0x00, 0x00, 0x56, 0x00, 0x00, 0x78};

IPAddress Eth_ipDestination;

unsigned int portMy = 5120;           // port of this module
unsigned int AOGNtripPort = 2233;     // port NTRIP data from AOG comes in
unsigned int AOGAutoSteerPort = 8888; // port Autosteer data from AOG comes in
unsigned int portDestination = 9999;  // Port of AOG that listens
// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Eth_udpPAOGI;     // Out port 5544
EthernetUDP Eth_udpNtrip;     // In port 2233
EthernetUDP Eth_udpAutoSteer; // In & Out Port 8888

extern bool Autosteer_running;
extern ConfigIP_t networkAddress;

char Eth_NTRIP_packetBuffer[SERIAL_BUFFER_SIZE]; // buffer for receiving ntrip data

/// @brief Initialization of Ethernet connection, called in setup()
/// @param
void EthernetStart(void)
{
    // start the Ethernet connection:
    Serial.println("Initializing ethernet with static IP address");

    // try to congifure using IP:
    Ethernet.begin(mac, 0); // Start Ethernet with IP 0.0.0.0

    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware)
    {
        Serial.println("Ethernet shield was not found. GPS via USB only.");

        return;
    }

    if (Ethernet.linkStatus() == LinkOFF)
    {
        Serial.println("Ethernet cable is not connected - Who cares we will start ethernet anyway.");
    }

    // grab the ip from EEPROM
    Eth_myip[0] = networkAddress.ipOne;
    Eth_myip[1] = networkAddress.ipTwo;
    Eth_myip[2] = networkAddress.ipThree;
    if (Autosteer_running)
    {
        Eth_myip[3] = 126; // 126 is steer module, with or without GPS
    }
    else
    {
        Eth_myip[3] = 120; // 120 is GPS only module
    }

    Ethernet.setLocalIP(Eth_myip); // Change IP address to IP set by user
    Serial.println("\r\nEthernet status OK");
    Serial.printf("IP set Manually: ");
    Serial.println(Ethernet.localIP());

    Eth_ipDestination[0] = Eth_myip[0];
    Eth_ipDestination[1] = Eth_myip[1];
    Eth_ipDestination[2] = Eth_myip[2];
    Eth_ipDestination[3] = 255;

    Serial.printf("\r\nEthernet IP of module: ");
    Serial.println(Ethernet.localIP());
    Serial.printf("Ethernet sending to IP: ");
    Serial.println(Eth_ipDestination);
    Serial.printf("All data sending to port: ");
    Serial.println(portDestination);

    // init UPD Port sending to AOG
    if (Eth_udpPAOGI.begin(portMy))
    {
        Serial.printf("Ethernet GPS UDP sending from port: ");
        Serial.println(portMy);
    }

    // init UPD Port getting NTRIP from AOG
    if (Eth_udpNtrip.begin(AOGNtripPort)) // AOGNtripPort
    {
        Serial.printf("Ethernet NTRIP UDP listening to port: ");
        Serial.println(AOGNtripPort);
    }

    // init UPD Port getting AutoSteer data from AOG
    if (Eth_udpAutoSteer.begin(AOGAutoSteerPort)) // AOGAutoSteerPortipPort
    {
        Serial.printf("Ethernet AutoSteer UDP listening to & send from port: ");
        Serial.println(AOGAutoSteerPort);
    }
}

extern HardwareSerialIMXRT *SerialGPS;
void Process_RTK_FromUDP(void)
{ // Check for RTK via UDP
    unsigned int packetLength = Eth_udpNtrip.parsePacket();

    if (packetLength > 0)
    {
        if (packetLength > SERIAL_BUFFER_SIZE) packetLength = SERIAL_BUFFER_SIZE;
        Eth_udpNtrip.read(Eth_NTRIP_packetBuffer, packetLength);
        SerialGPS->write(Eth_NTRIP_packetBuffer, packetLength);
    }
}

extern elapsedMillis EthernetCheck_msCounter;
void EthernetTask(void)
{
    // ethernet milisecond counter elapsed
    if (EthernetCheck_msCounter > 1000)
    {
        if (Ethernet.linkStatus() == LinkON)
        {
            EthernetCheck_msCounter = 0;
            digitalWrite(Ethernet_Active_LED, 1);
            digitalWrite(MY_LED_1, HIGH);
        }
        else
        {
            digitalWrite(Ethernet_Active_LED, 0);
            digitalWrite(MY_LED_1, LOW);
        }
    }
}