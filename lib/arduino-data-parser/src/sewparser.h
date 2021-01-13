#ifndef SEWPARSER
#define SEWPARSER
#include <Arduino.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "sewprotocol.h"

#define MAXBUFFER 512
#define MAXFRAMES 4

class SewParser {
    public:
        SewParser();
        int decodeFrame(uint8_t*, int);
        int decodeFrameWithCallback(uint8_t*, int, int (*callback)(FRAME, int));
        FRAME& getFrame(int);
        // Create Data frames
        static int encodeTemperature(FRAME&, uint8_t[], float);
        static int encodeGPS(FRAME&, uint8_t[], float lat, float lng, float alt);
        static int encodeHumidity(FRAME&, uint8_t[], float);
        static int encodeDistance(FRAME&, uint8_t[], float);
        static int encodeDCMotor(FRAME&, uint8_t[], uint8_t, uint8_t, uint8_t);
        static int encodeSwitch(FRAME&, uint8_t[], uint8_t);
        static int encodeToggle(FRAME&, uint8_t[], bool);
        // Create Commands frames
        static int encodeCMDTemperature(FRAME&, uint8_t[]);
        static int encodeCMDGPS(FRAME&, uint8_t[]);
        static int encodeCMDHumidity(FRAME&, uint8_t[]);
        static int encodeCMDDistance(FRAME&, uint8_t[]);
        static int encodeCMDDCMotor(FRAME&, uint8_t[]);
        static int encodeCMDSwitch(FRAME&, uint8_t[]);
        static int encodeCMDToggle(FRAME&, uint8_t[]);
        // Get Payloads
        static int getSingleFloatPayload(FRAME, float&);
        static int getSingleUInt8Payload(FRAME, uint8_t&);
        static int getBooleanPayload(FRAME, bool&);
        static int getSingleUInt16Payload(FRAME, uint16_t&);
        static int getTemperaturePayload(FRAME, float&);
        static int getHumidityPayload(FRAME, float&);
        static int getDistancePayload(FRAME, float&);
        static int getSwitchPayload(FRAME, uint8_t&);
        static int getTogglePayload(FRAME, bool&);
        static int getGPSPayload(FRAME, float[]);
        static int getDCMotorPayload(FRAME, uint8_t[]);
    private:
        uint8_t partialFrame[MAXBUFFER];
        int partialFrameSize;
        FRAME frames[MAXFRAMES];
        int totalFramesAllocated;
};
#endif
