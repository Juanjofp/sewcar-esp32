//#include "sewprotocol.h"
#include <sewparser.h>

void printFrameStruct(FRAME);
void printFrame(const char*, uint8_t [], int);
void printPayload(FRAME);

uint8_t mac1[8] = {0x02, 0x04, 0x0A, 0x0F, 0xAE, 0x0E, 0x04, 0x06};
uint8_t mac2[8] = {0x02, 0x04, 0x0A, 0x0F, 0xAE, 0x0E, 0x04, 0x06};

uint8_t CMDTEMPERATUREFRAME[] = {
    0x53, 0x45, 0x57, 0x01, 0x13,
    0x00, 0x02, 0x04, 0x0a, 0x0f,
    0xae, 0x0e, 0x04, 0x06, 0x01,
    0x00, 0x57, 0x45, 0x53
};

uint8_t DISTANCEFRAME[] = {
    0x53, 0x45, 0x57, 0x01,
    0x17, 0x00, 0x02, 0x04,
    0x0a, 0x0f, 0xae, 0x0e,
    0x04, 0x06, 0x03, 0x00,
    0x00, 0x00, 0xbc, 0x41,
    0x57, 0x45, 0x53
};

uint8_t GPSFRAME[] = {
    0x53, 0x45, 0x57, 0x01, 0x1f, 0x00, 0x02, 0x04,
    0x0a, 0x0f, 0xae, 0x0e, 0x04, 0x06, 0x0a, 0x00,
    0x6b, 0x7e, 0x14, 0x42, 0x68, 0xcd, 0x8f, 0xbf,
    0x00, 0x00, 0xd9, 0x42, 0x57, 0x45, 0x53
};

uint8_t GPSANDDISTANCEFRAME[] = {
    0x53, 0x45, 0x57, 0x01,
    0x17, 0x00, 0x02, 0x04,
    0x0a, 0x0f, 0xae, 0x0e,
    0x04, 0x06, 0x03, 0x00,
    0x00, 0x00, 0xbc, 0x41,
    0x57, 0x45, 0x53, 0x53, 0x45, 0x57, 0x01, 0x1f, 0x00, 0x02, 0x04,
    0x0a, 0x0f, 0xae, 0x0e, 0x04, 0x06, 0x0a, 0x00,
    0x6b, 0x7e, 0x14, 0x42, 0x68, 0xcd, 0x8f, 0xbf,
    0x00, 0x00, 0xd9, 0x42, 0x57, 0x45, 0x53
};

uint8_t DISTANCEANDHALFGPS[] = {
    0x53, 0x45, 0x57, 0x01,
    0x17, 0x00, 0x02, 0x04,
    0x0a, 0x0f, 0xae, 0x0e,
    0x04, 0x06, 0x03, 0x00,
    0x00, 0x00, 0xbc, 0x41,
    0x57, 0x45, 0x53, 0x53, 0x45, 0x57, 0x01, 0x1f, 0x00, 0x02, 0x04,
    0x0a, 0x0f, 0xae, 0x0e, 0x04, 0x06, 0x0a, 0x00
};

uint8_t HALFGPSANDDISTANCE[] = {
    0x6b, 0x7e, 0x14, 0x42, 0x68, 0xcd, 0x8f, 0xbf,
    0x00, 0x00, 0xd9, 0x42, 0x57, 0x45, 0x53, 0x53,
    0x45, 0x57, 0x01,
    0x17, 0x00, 0x02, 0x04,
    0x0a, 0x0f, 0xae, 0x0e,
    0x04, 0x06, 0x03, 0x00,
    0x00, 0x00, 0xbc, 0x41,
    0x57, 0x45, 0x53
};

void printFrame(const char* msg, uint8_t frame[], int size) {
    printf("%s [", msg);
    for(int i = 0; i < size; i++) {
        printf("%02x", frame[i]);
    }
    printf("]\n");
}

int printFrameStruct(FRAME frame, int status) {
    printf("\nFrame [\n");
    printFrame("Buffer: ", frame.frame, frame.size);
    printf("Version: %d\n", frame.version);
    printf("Size: %d of %d\n", frame.payloadSize, frame.size);
    printf("Type: %d\n", frame.type);
    printf("MAC: [");
    for(int i = 0; i < 8; i++) {
        printf("%02x", frame.mac[i]);
    }
    printf("]\n");
    printf("PAYLOAD: [");
    for(int i = 16; i < (16 + frame.payloadSize); i++) {
        printf("%02x", frame.frame[i]);
    }
    printf("]\n");
    printPayload(frame);
    printf("]\n");

    return 0;
}

void printPayload(FRAME frame) {
    switch(frame.type) {
        case GPS:
            float coords[4];
            SewParser::getGPSPayload(frame, coords);
            printf("GPS [%f, %f, %f]\n\n", coords[0], coords[1], coords[2]);
            break;
        case DCMOTOR:{
            uint8_t data[3];
            SewParser::getDCMotorPayload(frame, data);
            printf("DCMotor [%d, %d, %d]\n\n", data[0], data[1], data[2]);
            break;
        }
        case SWITCH: {
            uint8_t state;
            uint8_t& refState = state;
            SewParser::getSwitchPayload(frame, refState);
            printf("Switch [%d]\n\n", refState);
            break;
        }
        case TEMPERATURE:{
            float value;
            float &refValue = value;
            int isCmd = SewParser::getTemperaturePayload(frame, refValue);
            printf("Temperature (%d) [%f]\n\n", isCmd, refValue);
            break;
        }
        case HUMIDUTY:{
            float value;
            float &refValue = value;
            SewParser::getHumidityPayload(frame, refValue);
            printf("Humidity [%f]\n\n", refValue);
            break;
        }
        case DISTANCE:{
            float value;
            float &refValue = value;
            SewParser::getDistancePayload(frame, refValue);
            printf("DISTANCE [%f]\n\n", refValue);
            break;
        }
        default:
            printf("PAYLOAD: [");
            for(int i = 16; i < (16 + frame.payloadSize); i++) {
                printf("%02x", frame.frame[i]);
            }
            printf("]\n\n");
            break;
    }
}

int main(void) {
    FRAME tmpFrame, frame;
    int response;
    SewParser sp = SewParser();

    response = SewParser::encodeDCMotor(frame, mac1, 1, 1, 255);
    printFrame("MOTORDC", frame.frame, frame.size);

    response = SewParser::encodeDistance(frame, mac2, 23.5f);
    printFrame("DISTANCE", frame.frame, frame.size);

    response = SewParser::encodeHumidity(frame, mac1, 66);
    printFrame("HUMIDITY", frame.frame, frame.size);

    response = SewParser::encodeTemperature(frame, mac2, 32.5f);
    printFrame("TEMPERATURE", frame.frame, frame.size);

    response = SewParser::encodeGPS(frame, mac1, 37.123456, -1.123456, 108.50);
    printFrame("GPS", frame.frame, frame.size);

    response = SewParser::encodeSwitch(frame, mac1, 1);
    printFrame("SWITCH", frame.frame, frame.size);


    response = SewParser::encodeDCMotor(tmpFrame, mac1, 1, 0, 200);
    printFrameStruct(tmpFrame, 0);
    printPayload(tmpFrame);

    response = sp.decodeFrameWithCallback(GPSANDDISTANCEFRAME, sizeof(GPSANDDISTANCEFRAME), printFrameStruct);
    response = sp.decodeFrameWithCallback(DISTANCEANDHALFGPS, sizeof(DISTANCEANDHALFGPS), printFrameStruct);
    response = sp.decodeFrameWithCallback(HALFGPSANDDISTANCE, sizeof(HALFGPSANDDISTANCE), printFrameStruct);


    response = sp.decodeFrame(CMDTEMPERATUREFRAME, sizeof(CMDTEMPERATUREFRAME));
    for(int i = 0; i < response; i++) {
        printFrameStruct(sp.getFrame(i), 0);
    }

    response = sp.decodeFrame(DISTANCEFRAME, sizeof(DISTANCEFRAME));
    for(int i = 0; i < response; i++) {
        printFrameStruct(sp.getFrame(i), 0);
    }

    response = sp.decodeFrame(GPSANDDISTANCEFRAME, sizeof(GPSANDDISTANCEFRAME));
    for(int i = 0; i < response; i++) {
        printFrameStruct(sp.getFrame(i), 0);
    }

    response = sp.decodeFrame(DISTANCEANDHALFGPS, sizeof(DISTANCEANDHALFGPS));
    for(int i = 0; i < response; i++) {
        printFrameStruct(sp.getFrame(i), 0);
    }


    response = sp.decodeFrame(HALFGPSANDDISTANCE, sizeof(HALFGPSANDDISTANCE));
    for(int i = 0; i < response; i++) {
        printFrameStruct(sp.getFrame(i), 0);
    }
}
