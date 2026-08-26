
#include "Arduino.h"
#include <Wire.h>

#include "BNO_RVC.h"

//constructor destructor
BNO_rvc::BNO_rvc(void) {}
BNO_rvc::~BNO_rvc(void) {}

//set the object variables
bool BNO_rvc::begin(Stream *theSerial) {
  serial_dev = theSerial;
  return true;
}


//read the 19 byte sentence: AA AA Index(1) Yaw(2) Pitch(2) Roll(2) X(2) Y(2) Z(2) Reserved(3) Checksum(1)
//                                  |        |                       |
//                                  |        |                       > Acceleration
//                                  |        > Rotation around Z-axis since reset (0.01° increments, range ± 180°).
//                                  >          value x100. Jako zataceni
//                                  > increasing in sequence
//Data arrives every 10ms. Called every 100ms, so drain buffer and use newest valid packet.
BNO_rvcStatus_t BNO_rvc::read(BNO_rvcData* bnoData) {
    if (bnoData == NULL) return BNO_RVC_NULLPTR;

    int avail = serial_dev->available();
    if (avail < 19) return BNO_RVC_NOT_ENOUGH_DATA; // Not enough data for a packet

    // Read all available bytes into a local buffer to find the newest packet
    const int maxBuf = 64;  // serial has about 64 bytes
    static uint8_t raw[maxBuf];
    static uint8_t lastIndex = 0;


    for (int i = lastIndex; i < lastIndex + 19; i++) {
        raw[i] = serial_dev->read();
    }

    // Search valid 19-byte packet (header: 0xAA 0xAA)
    int packetStart = -1;
    for (int i = 0; i < lastIndex + 19; i++) {
        if (raw[i] == 0xAA && raw[i + 1] == 0xAA) {
            // Verify checksum: sum of bytes [2..17] == byte [18]
            packetStart = -2; // mark header found
            uint8_t sum = 0;
            for (int j = 2; j < 18; j++) {sum += raw[i + j];}
            //Serial.printf("Index %d, avail %d\r\n", raw[i + 2], avail);
            if (sum == raw[i + 18]) {
                packetStart = i;
                //Serial.printf("BNO08x full packet found at index %d\r\n", packetStart);
                break;
            }
            else
            {
                Serial.printf("BNO08x sum fail sum is %d !=%d\r\n", sum, raw[i + 18]);
            }
        }
    }

    lastIndex = (lastIndex + 19) % maxBuf;  // save last processed byte to continue reading from serial next time

    if (packetStart == -1)
    {
        return BNO_RVC_NO_VALID_HEADER; // No valid packet found
    }
    if (packetStart == -2)
    {
        return BNO_RVC_NO_VALID_CHSUM;  // Header found but checksum failed
    }

    /* packet complete */
    lastIndex = 0;

    // Point to payload (skip the two 0xAA header bytes + Index byte)
    uint8_t *buffer = &raw[packetStart + 2];

    int16_t temp;
    temp = buffer[1] + (buffer[2] << 8);    //load Yaw(Z axis) - zataceni

    bnoData->yawX100 = temp; //For angular velocity calc

    // load data to output structure, convert to degrees x10
    bnoData->yawX10 = (int16_t)((float)temp * DEGREE_SCALE);
    if (bnoData->yawX10 < 0) bnoData->yawX10 +=3600;

    temp = buffer[3] + (buffer[4] << 8);
    bnoData->pitchX10 = (int16_t)((float)temp * DEGREE_SCALE);

    temp = buffer[5] + (buffer[6] << 8);
    bnoData->rollX10 = (int16_t)((float)temp * DEGREE_SCALE);

    return BNO_RVC_SUCCESS; // Success
}
