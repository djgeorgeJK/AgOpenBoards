
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

//read the 19 byte sentence: AA AA Index Yaw(2) Pitch(2) Roll(2) X(2) Y(2) Z(2) Reserved Checksum
//Data arrives every 10ms. Called every 100ms, so drain buffer and use newest valid packet.
bool BNO_rvc::read(BNO_rvcData* bnoData) {
    if (!bnoData) return false;

    int avail = serial_dev->available();
    if (avail < 19) return false;

    // Read all available bytes into a local buffer to find the newest packet
    const int maxBuf = 256; // shall be for 16 packets, more than enough for 100ms at 10ms/packet
    uint8_t raw[maxBuf];
    int count = (avail > maxBuf) ? maxBuf : avail;

    // If more data than buffer, discard oldest bytes first
    while (avail > maxBuf) {
        serial_dev->read();
        avail--;
    }

    for (int i = 0; i < count; i++) {
        raw[i] = serial_dev->read();
    }

    // Search backwards for the last valid 19-byte packet (header: 0xAA 0xAA)
    int packetStart = -1;
    for (int i = count - 19; i >= 0; i--) {
        if (raw[i] == 0xAA && raw[i + 1] == 0xAA) {
            // Verify checksum: sum of bytes [2..17] == byte [18]
            uint8_t sum = 0;
            for (int j = 2; j < 18; j++) sum += raw[i + j];
            if (sum == raw[i + 18]) {
                packetStart = i;
                break;
            }
        }
    }

    if (packetStart < 0) return false;

    // Point to payload (skip the two 0xAA header bytes)
    uint8_t *buffer = &raw[packetStart + 2];

    int16_t temp;
    temp = buffer[1] + (buffer[2] << 8);

    if (angCounter < 20)
    {
        bnoData->yawX100 = temp; //For angular velocity calc
        bnoData->angVel += (temp - prevYAw);
        angCounter++;
        prevYAw = temp;
    }
    else
    {
        angCounter = 0;
        prevYAw = bnoData->angVel = 0;
    }

    bnoData->yawX10 = (int16_t)((float)temp * DEGREE_SCALE);
    if (bnoData->yawX10 < 0) bnoData->yawX10 +=3600;

    temp = buffer[3] + (buffer[4] << 8);
    bnoData->pitchX10 = (int16_t)((float)temp * DEGREE_SCALE);

    temp = buffer[5] + (buffer[6] << 8);
    bnoData->rollX10 = (int16_t)((float)temp * DEGREE_SCALE);

    return true;
}
