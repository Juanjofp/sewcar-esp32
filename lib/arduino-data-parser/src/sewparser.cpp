#include "sewparser.h"

void printHex(const uint8_t* buffer, int size) {
    printf("\n[");
    for(int i = 0; i < size; i++) {
        printf("%x", buffer[i]);
    };
    printf("]\n");
}

SewParser::SewParser() {
    //printf("Constructor SewParser\n");
    this->partialFrameSize = 0;
    this->totalFramesAllocated = 0;
}

int SewParser::decodeFrame(uint8_t* frame, int size) {
    int totalSize = size + this->partialFrameSize;
    uint8_t totalFrame[totalSize];
    int indexHeader = -1, indexTail = -1;

    if(totalSize < MINFRAMESIZE || totalSize > MAXBUFFER) {
        memcpy(this->partialFrame + this->partialFrameSize, frame, size);
        this->partialFrameSize = this->partialFrameSize + size;
        return -1;
    }

    memcpy(totalFrame, this->partialFrame, this->partialFrameSize);
    memcpy(totalFrame + this->partialFrameSize, frame, size);
    //printHex(totalFrame, totalSize);
    this->partialFrameSize = 0;
    this->totalFramesAllocated = 0;
    for(int i = 0; i < totalSize; i++) {
        uint8_t xbyte = totalFrame[i];
        //printf("%x", xbyte);
        if(xbyte == 0x53) {
            if((i + 2) <= totalSize) {
                if(
                    totalFrame[i + 2] == 0x57 && totalFrame[i + 1] == 0x45
                ) {
                    //printf("\nHeader detected at %d %x %x\n", i, totalFrame[i - 1], totalFrame[i - 2]);
                    indexHeader = i;
                    // Search size in buffer
                    VALUE_UINT16 frameSize;
                    memcpy(frameSize.bytes, totalFrame + INDEXSIZE + i, 2);
                    i += (frameSize.value - 1);
                }
            }
        }

        if(xbyte == 0x53 && indexHeader > -1) {
            if((i - 2) >= 0) {
                if(
                    totalFrame[i - 2] == 0x57 && totalFrame[i - 1] == 0x45
                ) {
                    //printf("\nTail detected at %d %x %x\n", i, totalFrame[i - 1], totalFrame[i - 2]);
                    indexTail = i;
                    int frameSize = (indexTail - indexHeader) > MAXFRAMESIZE ? MAXFRAMESIZE : (indexTail - indexHeader + 1);

                    memcpy(this->frames[this->totalFramesAllocated].frame, totalFrame + indexHeader, frameSize);
                    this->frames[this->totalFramesAllocated].size = frameSize;
                    this->totalFramesAllocated += 1;
                    indexHeader = -1;
                    indexTail = -1;

                    if(this->totalFramesAllocated == MAXFRAMES && totalSize > i) {
                        int rest = totalSize - i;
                        memcpy(this->partialFrame, totalFrame + i, rest);
                        this->partialFrameSize = rest;
                        break;
                    }
                }
            }
        }
    }
    if(indexHeader >= 0 && indexTail == -1) {
        int rest = totalSize - indexHeader;
        memcpy(this->partialFrame, totalFrame + indexHeader, rest);
        this->partialFrameSize = rest;
    }
    //printf("\n Partial: %d ", this->partialFrameSize);
    //printHex(this->partialFrame, this->partialFrameSize);
    return this->totalFramesAllocated;
}

int SewParser::decodeFrameWithCallback(uint8_t *frame, int size, int (*callback)(FRAME, int)) {
    int totalSize = size + this->partialFrameSize;
    uint8_t totalFrame[totalSize];
    int indexHeader = -1, indexTail = -1;

    if(totalSize < MINFRAMESIZE || totalSize > MAXBUFFER) {
        memcpy(this->partialFrame + this->partialFrameSize, frame, size);
        this->partialFrameSize = this->partialFrameSize + size;
        return -1;
    }

    memcpy(totalFrame, this->partialFrame, this->partialFrameSize);
    memcpy(totalFrame + this->partialFrameSize, frame, size);
    //printHex(totalFrame, totalSize);
    this->partialFrameSize = 0;
    this->totalFramesAllocated = 0;
    for(int i = 0; i < totalSize; i++) {
        uint8_t xbyte = totalFrame[i];
        //printf("%x", xbyte);
        if(xbyte == 0x53) {
            if((i + 2) <= totalSize) {
                if(
                    totalFrame[i + 2] == 0x57 && totalFrame[i + 1] == 0x45
                ) {
                    //printf("\nHeader detected at %d %x %x\n", i, totalFrame[i - 1], totalFrame[i - 2]);
                    indexHeader = i;
                    // Search size in buffer
                    VALUE_UINT16 frameSize;
                    memcpy(frameSize.bytes, totalFrame + INDEXSIZE + i, 2);
                    i += (frameSize.value - 1);
                    //printf("Jump to %d %x of %d step %d", i, totalFrame[i], totalSize, frameSize.value - 1);
                }
            }
        }

        if(xbyte == 0x53 && indexHeader > -1) {
            if((i - 2) >= 0) {
                if(
                    totalFrame[i - 2] == 0x57 && totalFrame[i - 1] == 0x45
                ) {
                    //printf("\nTail detected at %d %x %x\n", i, totalFrame[i - 1], totalFrame[i - 2]);
                    indexTail = i;
                    int frameSize = (indexTail - indexHeader) > MAXFRAMESIZE ? MAXFRAMESIZE : (indexTail - indexHeader + 1);

                    FRAME tmpFrame;
                    memcpy(tmpFrame.frame, totalFrame + indexHeader, frameSize);
                    tmpFrame.size = frameSize;
                    int response = extendFrame(tmpFrame);
                    callback(tmpFrame, response);

                    this->totalFramesAllocated += 1;
                    indexHeader = -1;
                    indexTail = -1;
                }
            }
        }
    }
    if(indexHeader >= 0 && indexTail == -1) {
        int rest = totalSize - indexHeader;
        memcpy(this->partialFrame, totalFrame + indexHeader, rest);
        this->partialFrameSize = rest;
    }
    //printf("\n Partial: %d ", this->partialFrameSize);
    //printHex(this->partialFrame, this->partialFrameSize);
    return this->totalFramesAllocated;
}

FRAME& SewParser::getFrame(int index) {
    if(index < 0 || index >= MAXFRAMES) {
        index = 0;
    }
    extendFrame(this->frames[index]);
    return this->frames[index];
}

int SewParser::encodeTemperature(FRAME& frame, uint8_t mac[], float value) {
    return prepareTemperatureFrame(frame, mac, value);
}

int SewParser::encodeHumidity(FRAME& frame, uint8_t mac[], float value) {
    return prepareHumidityFrame(frame, mac, value);
}

int SewParser::encodeDistance(FRAME& frame, uint8_t mac[], float value) {
    return prepareDistanceFrame(frame, mac, value);
}

int SewParser::encodeGPS(FRAME& frame, uint8_t mac[], float lat, float lng, float alt) {
    return prepareGPSFrame(frame, mac, lat, lng, alt);
}

int SewParser::encodeDCMotor(FRAME& frame, uint8_t mac[], uint8_t enabled, uint8_t reverse, uint8_t power) {
    return prepareDCMotorFrame(frame, mac, enabled, reverse, power);
}

int SewParser::encodeSwitch(FRAME& frame, uint8_t mac[], uint8_t enabled) {
    return prepareSwitchFrame(frame, mac, enabled);
}

int SewParser::encodeToggle(FRAME& frame, uint8_t mac[], bool enabled) {
    return prepareToggleFrame(frame, mac, enabled);
}

int SewParser::encodeCMDTemperature(FRAME& frame, uint8_t mac[]) {
    return prepareEmptyTemperatureFrame(frame, mac);
}

int SewParser::encodeCMDGPS(FRAME& frame, uint8_t mac[]) {
    return prepareEmptyGPSFrame(frame, mac);
}

int SewParser::encodeCMDHumidity(FRAME& frame, uint8_t mac[]) {
    return prepareEmptyHumidityFrame(frame, mac);
}

int SewParser::encodeCMDDistance(FRAME& frame, uint8_t mac[]) {
    return prepareEmptyDistanceFrame(frame, mac);
}

int SewParser::encodeCMDDCMotor(FRAME& frame, uint8_t mac[]) {
    return prepareEmptyDCMotorFrame(frame, mac);
}

int SewParser::encodeCMDSwitch(FRAME& frame, uint8_t mac[]) {
    return prepareEmptySwitchFrame(frame, mac);
}

int SewParser::encodeCMDToggle(FRAME& frame, uint8_t mac[]) {
    return prepareEmptyToggleFrame(frame, mac);
}

int SewParser::getGPSPayload(FRAME frame, float coords[]) {
    return decodeGPSPayload(frame, coords);
}

int SewParser::getDCMotorPayload(FRAME frame, uint8_t data[]){
    return decodeDCMotorPayload(frame, data);
}

int SewParser::getSingleFloatPayload(FRAME frame, float &value) {
    return decodeSingleFloatPayload(frame, value);
}

int SewParser::getSingleUInt8Payload(FRAME frame, uint8_t &value) {
    return decodeSingleUInt8Payload(frame, value);
}

int SewParser::getBooleanPayload(FRAME frame, bool &value) {
    return decodeBooleanPayload(frame, value);
}

int SewParser::getSingleUInt16Payload(FRAME frame, uint16_t &value) {
    return decodeSingleUInt16Payload(frame, value);
}

int SewParser::getTemperaturePayload(FRAME frame, float &value) {
    return decodeTemperaturePayload(frame, value);
}

int SewParser::getHumidityPayload(FRAME frame, float &value) {
    return decodeHumidityPayload(frame, value);
}

int SewParser::getDistancePayload(FRAME frame, float &value) {
    return decodeDistancePayload(frame, value);
}

int SewParser::getSwitchPayload(FRAME frame, uint8_t &value) {
    return decodeSwitchPayload(frame, value);
}

int SewParser::getTogglePayload(FRAME frame, bool &value) {
    return decodeTogglePayload(frame, value);
}
