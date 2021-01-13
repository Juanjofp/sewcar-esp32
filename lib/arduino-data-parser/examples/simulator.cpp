#include <sewparser.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>

#define PORT 6966
#define HOST "127.0.0.1"

void printFrameStruct(FRAME);
void printFrame(const char*, uint8_t [], int);
void printPayload(FRAME);
int intRandom(int, int);
int generateValues(int) ;

uint8_t macTemperature[8] = {0xAA, 0x04, 0x0A, 0x0F, 0xAE, 0x0E, 0x04, 0x01};
uint8_t macHumidity[8] = {0xAA, 0x04, 0x0A, 0x0F, 0xAE, 0x0E, 0x04, 0x02};
uint8_t macDistance[8] = {0xAA, 0x04, 0x0A, 0x0F, 0xAE, 0x0E, 0x04, 0x03};
uint8_t macGPS[8] = {0xAA, 0x04, 0x0A, 0x0F, 0xAE, 0x0E, 0x04, 0x04};
uint8_t macDCMotor[8] = {0xAA, 0x04, 0x0A, 0x0F, 0xAE, 0x0E, 0x04, 0x05};
uint8_t macSwitch[8] = {0xAA, 0x04, 0x0A, 0x0F, 0xAE, 0x0E, 0x04, 0x06};

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
            printf("GPS [%f, %f, %f]\n----------------------------\n", coords[0], coords[1], coords[2]);
            break;
        case DCMOTOR:{
            uint8_t data[3];
            SewParser::getDCMotorPayload(frame, data);
            printf("DCMotor [%d, %d, %d]\n----------------------------\n", data[0], data[1], data[2]);
            break;
        }
        case SWITCH: {
            uint8_t state;
            uint8_t& refState = state;
            SewParser::getSwitchPayload(frame, refState);
            printf("Switch [%d]\n----------------------------\n", refState);
            break;
        }
        case TOGGLE: {
            bool state;
            bool& refState = state;
            SewParser::getTogglePayload(frame, refState);
            printf("Toggle [%c]\n----------------------------\n", refState ? 'T' : 'F');
            break;
        }
        case TEMPERATURE:{
            float value;
            float &refValue = value;
            int isCmd = SewParser::getTemperaturePayload(frame, refValue);
            printf("Temperature (%d) [%f]\n----------------------------\n", isCmd, refValue);
            break;
        }
        case HUMIDUTY:{
            float value;
            float &refValue = value;
            SewParser::getHumidityPayload(frame, refValue);
            printf("Humidity [%f]\n----------------------------\n", refValue);
            break;
        }
        case DISTANCE:{
            float value;
            float &refValue = value;
            SewParser::getDistancePayload(frame, refValue);
            printf("DISTANCE [%f]\n----------------------------\n", refValue);
            break;
        }
        default:
            printf("UNKNOWN PAYLOAD: [");
            for(int i = 16; i < (16 + frame.payloadSize); i++) {
                printf("%02x", frame.frame[i]);
            }
            printf("]\n----------------------------\n");
            break;
    }
}

int intRandom(int min, int max) {
    int r;
    const unsigned int range = 1 + max - min;
    const unsigned int buckets = RAND_MAX / range;
    const unsigned int limit = buckets * range;

    /* Create equal size buckets all in a row, then fire randomly towards
     * the buckets until you land in one of them. All buckets are equally
     * likely. If you land off the end of the line of buckets, try again. */
    do
    {
        r = rand();
    } while (r >= limit);

    return min + (r / buckets);
}

int generateValues(int sock) {
    FRAME frame;
    int response;

    response = SewParser::encodeDCMotor(frame, macDCMotor, intRandom(0, 1), intRandom(0, 1), intRandom(0, 255));
    printFrame("MOTORDC", frame.frame, frame.size);
    send(sock , frame.frame, frame.size, 0 );
    response = SewParser::encodeDistance(frame, macDistance, ((float)intRandom(0, 500)/10));
    printFrame("DISTANCE", frame.frame, frame.size);
    send(sock , frame.frame, frame.size, 0 );
    response = SewParser::encodeHumidity(frame, macHumidity, ((float)intRandom(0, 1000)/10));
    printFrame("HUMIDITY", frame.frame, frame.size);
    send(sock , frame.frame, frame.size, 0 );
    response = SewParser::encodeTemperature(frame, macTemperature, ((float)intRandom(-100, 500)/10));
    printFrame("TEMPERATURE", frame.frame, frame.size);
    send(sock , frame.frame, frame.size, 0 );
    response = SewParser::encodeGPS(frame, macGPS, ((float)intRandom(-90000000, 90000000)/1000000), ((float)intRandom(-90000000, 90000000)/1000000), ((float)intRandom(0, 50000)/10));
    printFrame("GPS", frame.frame, frame.size);
    send(sock , frame.frame, frame.size, 0 );
    response = SewParser::encodeSwitch(frame, macTemperature, intRandom(0, 1));
    printFrame("SWITCH", frame.frame, frame.size);
    send(sock , frame.frame, frame.size, 0 );
    response = SewParser::encodeToggle(frame, macTemperature, intRandom(0, 1) == 0 ? false : true);
    printFrame("TOGGLE", frame.frame, frame.size);
    send(sock , frame.frame, frame.size, 0 );
    // response = SewParser::encodeDCMotor(tmpFrame, mac1, 1, 0, 200);
    // printFrameStruct(tmpFrame, 0);
    // printPayload(tmpFrame);

    // response = sp.decodeFrameWithCallback(GPSANDDISTANCEFRAME, sizeof(GPSANDDISTANCEFRAME), printFrameStruct);
    return 0;
}

int main(void) {
    SewParser sp;
    struct sockaddr_in address;
    int sock = 0, valread;
    struct sockaddr_in serv_addr;
    uint8_t buffer[1024] = {0x00};
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Socket creation error \n");
        return -1;
    }

    memset(&serv_addr, '0', sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET, HOST, &serv_addr.sin_addr)<=0)
    {
        printf("\nInvalid address/ Address not supported \n");
        return -1;
    }

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        printf("\nConnection Failed \n");
        return -1;
    }

    while(true){
        generateValues(sock);
        valread = read( sock , buffer, 512);
        printFrame("Buffer", buffer, valread);
        printf("Readed %d", valread);
        sp.decodeFrameWithCallback(buffer, valread, printFrameStruct);
        sleep(10);
    };

    return 0;
}
