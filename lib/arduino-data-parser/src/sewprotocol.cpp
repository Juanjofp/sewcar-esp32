#include "sewprotocol.h"

/*
 * Trama SEW:

Header: 3 bytes '0x530x450x57'

Version: 1 byte '0x01

MAC: 8 bytes Id del sensor

TYPE: 2 bytes 0x0001 -> Temperature, 0x000A -> GPS
Para errores usamos el Type 0xFFFF y su payload llevará el código de error

PAYLOAD: upto 112 bytes 0x41c4cccd, 0x4214f15b 0xbfabd93c 0x42d80000

Tail: 3 bytes '0x570x450x53'
 */
int prepareHumidityFrame(FRAME &frame, uint8_t mac[], float value) {
    return prepareFloatFrame(frame, mac, HUMIDUTY, value);
}

int prepareTemperatureFrame(FRAME &frame, uint8_t mac[], float value) {
    return prepareFloatFrame(frame, mac, TEMPERATURE, value);
}

int prepareDistanceFrame(FRAME &frame, uint8_t mac[], float value) {
    return prepareFloatFrame(frame, mac, DISTANCE, value);
}

int prepareFloatFrame(FRAME &frame, uint8_t mac[], uint16_t type, float value) {
    VALUE_FLOAT payload;
    payload.value = value;
    return prepareV1Frame(frame, mac, type, payload.bytes, sizeof(value));
}

int prepareGPSFrame(FRAME &frame, uint8_t mac[], float lat, float lng, float alt) {
    // PAYLOAD
    VALUE_FLOAT latitude;
    VALUE_FLOAT longitude;
    VALUE_FLOAT altitude;
    uint8_t payload[12];

    latitude.value = lat;
    memcpy(payload, latitude.bytes, 4);

    longitude.value = lng;
    memcpy(payload + 4, longitude.bytes, 4);

    altitude.value = alt;
    memcpy(payload + 8, altitude.bytes, 4);

    return prepareV1Frame(frame, mac, GPS, payload, sizeof(float) * 3);
}

int prepareDCMotorFrame(FRAME &frame, uint8_t mac[], uint8_t enabled, uint8_t reverse, uint8_t power) {
    // PAYLOAD
    uint8_t payload[3];
    payload[0] = enabled;
    payload[1] = reverse;
    payload[2] = power;

    return prepareV1Frame(frame, mac, DCMOTOR, payload, sizeof(enabled)*3);
}

int prepareSwitchFrame(FRAME &frame, uint8_t mac[], uint8_t status) {
    // PAYLOAD
    uint8_t payload[1];
    payload[0] = status;

    return prepareV1Frame(frame, mac, SWITCH, payload, sizeof(status));
}

int prepareToggleFrame(FRAME &frame, uint8_t mac[], bool enabled) {
    // PAYLOAD
    uint8_t payload[1];
    payload[0] = enabled ? 1 : 0;

    return prepareV1Frame(frame, mac, TOGGLE, payload, sizeof(uint8_t));
}

int prepareV1Frame(FRAME &frame, uint8_t* mac, uint16_t type, uint8_t* payload, uint16_t size) {
    return prepareFrame(frame, 0x01, mac, type, payload, size);
}

int prepareFrame(FRAME &frame, uint8_t version, uint8_t* mac, uint16_t type, uint8_t* payload, uint16_t size) {
    uint16_t totalSize = size + FRAMESIZE;

    if(totalSize > MAXFRAMESIZE) {
        return ERRORMAXSIZE;
    }

    // Header
    frame.frame[0] = 0x53;
    frame.frame[1] = 0x45;
    frame.frame[2] = 0x57;
    // Version
    frame.frame[3] = version;
    // SIZE
    VALUE_UINT16 frameSize;
    frameSize.value = totalSize;
    memcpy(frame.frame + 4, frameSize.bytes, 2);
    // MAC
    memcpy(frame.frame + 6, mac, 8);
    // TYPE
    SENSOR_TYPE typeOfSensor;
    typeOfSensor.value = type;
    memcpy(frame.frame + 14, typeOfSensor.bytes, 2);
    // PAYLOAD
    memcpy(frame.frame + 16, payload, size);
    // Tail
    frame.frame[totalSize - 3] = 0x57;
    frame.frame[totalSize - 2] = 0x45;
    frame.frame[totalSize - 1] = 0x53;

    frame.version = version;
    frame.size = totalSize;
    memcpy(frame.mac, mac, 8);
    frame.type = type;
    frame.payloadSize = size;

    return RETURNSUCCESS;
}

int prepareEmptyGPSFrame(FRAME &frame, uint8_t* mac) {
    return prepareEmptyFrame(frame, 0x01, mac, GPS);
}

int prepareEmptyHumidityFrame(FRAME &frame, uint8_t* mac) {
    return prepareEmptyFrame(frame, 0x01, mac, HUMIDUTY);
}

int prepareEmptyTemperatureFrame(FRAME &frame, uint8_t* mac) {
    return prepareEmptyFrame(frame, 0x01, mac, TEMPERATURE);
}

int prepareEmptyDistanceFrame(FRAME &frame, uint8_t* mac) {
    return prepareEmptyFrame(frame, 0x01, mac, DISTANCE);
}

int prepareEmptyDCMotorFrame(FRAME &frame, uint8_t* mac) {
    return prepareEmptyFrame(frame, 0x01, mac, DCMOTOR);
}

int prepareEmptySwitchFrame(FRAME &frame, uint8_t* mac) {
    return prepareEmptyFrame(frame, 0x01, mac, SWITCH);
}

int prepareEmptyToggleFrame(FRAME &frame, uint8_t* mac) {
    return prepareEmptyFrame(frame, 0x01, mac, TOGGLE);
}

int prepareEmptyFrame(FRAME &frame, uint8_t version, uint8_t* mac, uint16_t type) {
    uint16_t totalSize = FRAMESIZE;

    if(totalSize > MAXFRAMESIZE) {
        return ERRORMAXSIZE;
    }

    // Header
    frame.frame[0] = 0x53;
    frame.frame[1] = 0x45;
    frame.frame[2] = 0x57;
    // Version
    frame.frame[3] = version;
    // SIZE
    VALUE_UINT16 frameSize;
    frameSize.value = totalSize;
    memcpy(frame.frame + 4, frameSize.bytes, 2);
    // MAC
    memcpy(frame.frame + 6, mac, 8);
    // TYPE
    SENSOR_TYPE typeOfSensor;
    typeOfSensor.value = type;
    memcpy(frame.frame + 14, typeOfSensor.bytes, 2);
    // Tail
    frame.frame[totalSize - 3] = 0x57;
    frame.frame[totalSize - 2] = 0x45;
    frame.frame[totalSize - 1] = 0x53;

    frame.version = version;
    frame.size = totalSize;
    memcpy(frame.mac, mac, 8);
    frame.type = type;
    frame.payloadSize = 0;

    return RETURNSUCCESS;
}

int createFrame(FRAME& frame, const uint8_t* buffer, uint16_t size) {
    if(size < FRAMESIZE) {
        return ERRORMINSIZE;
    }
    if(size > MAXFRAMESIZE) {
        return ERRORMAXSIZE;
    }

    int payloadSize = size - FRAMESIZE;
    memcpy(frame.frame, buffer, size);
    frame.version = buffer[3];
    VALUE_UINT16 totalSize;
    memcpy(totalSize.bytes, buffer + 4, 2);
    if(totalSize.value != size) {
        return ERRORSIZENOMATCH;
    }
    frame.size = totalSize.value;
    memcpy(frame.mac, buffer + 6, 8);
    SENSOR_TYPE type;
    memcpy(type.bytes, buffer + 14, 2);
    frame.type = type.value;
    frame.payloadSize = payloadSize;

    return RETURNSUCCESS;
}

int extendFrame(FRAME& frame) {
    if(frame.size < FRAMESIZE) {
        return ERRORMINSIZE;
    }
    if(frame.size > MAXFRAMESIZE) {
        return ERRORMAXSIZE;
    }
    int payloadSize = frame.size - FRAMESIZE;
    frame.version = frame.frame[3];
    VALUE_UINT16 totalSize;
    memcpy(totalSize.bytes, frame.frame + 4, 2);
    if(totalSize.value != frame.size) {
        return ERRORSIZENOMATCH;
    }
    frame.size = totalSize.value;
    memcpy(frame.mac, frame.frame + 6, 8);
    SENSOR_TYPE type;
    memcpy(type.bytes, frame.frame + 14, 2);
    frame.type = type.value;
    frame.payloadSize = payloadSize;

    return RETURNSUCCESS;
}

int decodeGPSPayload(FRAME frame, float coords[]) {
    if(frame.type != GPS) {
        return ERRORINVALIDPAYLOAD;
    }
    if(frame.payloadSize == 0) {
        return ERRORCMDPAYLOAD;
    }

    VALUE_FLOAT lat, lng, alt;
    memcpy(lat.bytes, frame.frame + INDEXPAYLOAD, 4);
    memcpy(lng.bytes, frame.frame + INDEXPAYLOAD + 4, 4);
    memcpy(alt.bytes, frame.frame + INDEXPAYLOAD + 8, 4);

    coords[0] = lat.value;
    coords[1] = lng.value;
    coords[2] = alt.value;
    return RETURNSUCCESS;
}

int decodeDCMotorPayload(FRAME frame, uint8_t data[]) {
    if(frame.type != DCMOTOR) {
        return ERRORINVALIDPAYLOAD;
    }
    if(frame.payloadSize == 0) {
        return ERRORCMDPAYLOAD;
    }

    data[0] = frame.frame[INDEXPAYLOAD];
    data[1] = frame.frame[INDEXPAYLOAD + 1];
    data[2] = frame.frame[INDEXPAYLOAD + 2];

    return RETURNSUCCESS;
}

int decodeSingleFloatPayload(FRAME frame, float &value) {
    if(frame.payloadSize == 0) {
        return ERRORCMDPAYLOAD;
    }
    VALUE_FLOAT buffValue;
    memcpy(buffValue.bytes, frame.frame + INDEXPAYLOAD, 4);
    value = buffValue.value;

    return RETURNSUCCESS;
}

int decodeSingleUInt16Payload(FRAME frame, uint16_t &value) {
    if(frame.payloadSize == 0) {
        return ERRORCMDPAYLOAD;
    }
    VALUE_UINT16 buffValue;
    memcpy(buffValue.bytes, frame.frame + INDEXPAYLOAD, 2);
    value = buffValue.value;

    return RETURNSUCCESS;
}

int decodeSingleUInt8Payload(FRAME frame, uint8_t &value) {
    if(frame.payloadSize == 0) {
        return ERRORCMDPAYLOAD;
    }
    value = frame.frame[INDEXPAYLOAD];

    return RETURNSUCCESS;
}

int decodeBooleanPayload(FRAME frame, bool &value) {
    if(frame.payloadSize == 0) {
        return ERRORCMDPAYLOAD;
    }
    value = frame.frame[INDEXPAYLOAD] == 0 ? false : true;

    return RETURNSUCCESS;
}

int decodeTemperaturePayload(FRAME frame, float &value) {
    if(frame.type != TEMPERATURE) {
        return ERRORINVALIDPAYLOAD;
    }
    return decodeSingleFloatPayload(frame, value);
}

int decodeHumidityPayload(FRAME frame, float &value) {
    if(frame.type != HUMIDUTY) {
        return ERRORINVALIDPAYLOAD;
    }
    return decodeSingleFloatPayload(frame, value);
}

int decodeDistancePayload(FRAME frame, float &value) {
    if(frame.type != DISTANCE) {
        return ERRORINVALIDPAYLOAD;
    }
    return decodeSingleFloatPayload(frame, value);
}

int decodeSwitchPayload(FRAME frame, uint8_t &value) {
    if(frame.type != SWITCH) {
        return ERRORINVALIDPAYLOAD;
    }
    return decodeSingleUInt8Payload(frame, value);
}

int decodeTogglePayload(FRAME frame, bool &value) {
    if(frame.type != TOGGLE) {
        return ERRORINVALIDPAYLOAD;
    }
    return decodeBooleanPayload(frame, value);
}
