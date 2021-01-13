#ifndef SEWPROTOCOL
#define SEWPROTOCOL
#include <Arduino.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define MAXFRAMESIZE 32
#define MINFRAMESIZE 19
#define FRAMESIZE 19
#define MACSIZE 8
#define H1 0x53
#define H2 0x45
#define H3 0x57
#define INDEXSIZE 4
#define INDEXPAYLOAD 16

#define RETURNSUCCESS 0
#define ERRORMAXSIZE -1
#define ERRORMINSIZE -2
#define ERRORSIZENOMATCH -3
#define ERRORINVALIDPAYLOAD -4
#define ERRORCMDPAYLOAD -5

#define TEMPERATURE 0x0001
#define HUMIDUTY 0x0002
#define DISTANCE 0x0003
#define GPS 0x000A
#define DCMOTOR 0x0010
#define SWITCH 0x0011
#define TOGGLE 0x0012

// Floats values
typedef union value_float {
  uint8_t bytes[4];
  float value;
} VALUE_FLOAT;

// UINT16 values
typedef union value_uint16 {
  uint8_t bytes[4];
  uint16_t value;
} VALUE_UINT16;

// Int16 values
typedef union value_int16 {
  uint8_t bytes[4];
  int16_t value;
} VALUE_INT16;

// Sensor type
typedef union sensor_type {
  uint8_t bytes[2];
  uint16_t value;
} SENSOR_TYPE;

//Frame values (frame + size)
typedef struct struct_frame {
    uint8_t frame[MAXFRAMESIZE];
    uint8_t version;
    uint16_t size;
    uint8_t mac[MACSIZE];
    uint16_t type;
    uint16_t payloadSize;
} FRAME;

// Create Data frames
int prepareGPSFrame(FRAME&, uint8_t[], float lat, float lng, float alt);
int prepareHumidityFrame(FRAME&, uint8_t[], float);
int prepareTemperatureFrame(FRAME&, uint8_t[], float);
int prepareDistanceFrame(FRAME&, uint8_t[], float);
int prepareDCMotorFrame(FRAME&, uint8_t[], uint8_t, uint8_t, uint8_t);
int prepareSwitchFrame(FRAME&, uint8_t[], uint8_t);
int prepareToggleFrame(FRAME&, uint8_t[], bool);
// Create Commands frames
int prepareEmptyGPSFrame(FRAME&, uint8_t[]);
int prepareEmptyHumidityFrame(FRAME&, uint8_t[]);
int prepareEmptyTemperatureFrame(FRAME&, uint8_t[]);
int prepareEmptyDistanceFrame(FRAME&, uint8_t[]);
int prepareEmptyDCMotorFrame(FRAME&, uint8_t[]);
int prepareEmptySwitchFrame(FRAME&, uint8_t[]);
int prepareEmptyToggleFrame(FRAME&, uint8_t[]);
// helpers to create frames
int prepareFloatFrame(FRAME&, uint8_t[], uint16_t, float);
int prepareFrame(FRAME&, uint8_t, uint8_t[], uint16_t, uint8_t[], uint16_t);
int prepareEmptyFrame(FRAME&, uint8_t, uint8_t[], uint16_t);
int prepareV1Frame(FRAME&, uint8_t[], uint16_t, uint8_t*, uint16_t);
// Create frame from buffer
int createFrame(FRAME&, const uint8_t[], int);
int extendFrame(FRAME&);
// Decode payloads
int decodeSingleFloatPayload(FRAME, float&);
int decodeSingleUInt8Payload(FRAME, uint8_t&);
int decodeBooleanPayload(FRAME, bool&);
int decodeSingleUInt16Payload(FRAME, uint16_t&);
int decodeTemperaturePayload(FRAME, float&);
int decodeHumidityPayload(FRAME, float&);
int decodeDistancePayload(FRAME, float&);
int decodeSwitchPayload(FRAME, uint8_t&);
int decodeTogglePayload(FRAME, bool&);
int decodeGPSPayload(FRAME, float[]);
int decodeDCMotorPayload(FRAME, uint8_t[]);
#endif
