#include <Arduino.h>
#include <sewparser.h>
#include <sew-wifi-config.h>
#include <sew-web-server.hpp>
#include <sew-mqtt.hpp>
#include "sewdistance.hpp"
#include "sewmotors.hpp"
#include "sewpir.hpp"

#define SEW_PARSER_MAX_BUFFER_SIZE 120

#define MAX_DISTANCE 200
#define trigPinBack 17
#define echoPinBack 5
#define trigPinFront 18
#define echoPinFront 19

#define pirPin 16

#define enableM1 32
#define forwardM1 33
#define reverseM1 25

#define enabledM2 14
#define forwardM2 27
#define reverseM2 26

// Webserver and MQTT
SewWebServer sewServer;
SewMQTT sewMQTT;                                                                                            


uint8_t MML[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
uint8_t MMR[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02};
uint8_t MDF[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03};
uint8_t MDB[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04};
uint8_t MPIR[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05};

uint8_t* macMotorLeft = WiFi.macAddress(MML);
uint8_t* macMotorRight = WiFi.macAddress(MMR);
uint8_t* macDistanceFront = WiFi.macAddress(MDF);
uint8_t* macDistanceBack = WiFi.macAddress(MDB);
uint8_t* macPIR = WiFi.macAddress(MPIR);

// SEW Distance sensors
SewDistance distanceFront(trigPinFront, echoPinFront, MAX_DISTANCE);
SewDistance distanceBack(trigPinBack, echoPinBack, MAX_DISTANCE);
void distanceFrontCallback(int distance) {
  FRAME frame;
  SewParser::encodeDistance(frame, macDistanceFront, distance);
  sewMQTT.publish(0, frame.frame, frame.size);
}
void distanceBackCallback(int distance) {
  FRAME frame;
  SewParser::encodeDistance(frame, macDistanceBack, distance);
  sewMQTT.publish(0, frame.frame, frame.size);
}

// PIR Sensor
SewPIR sensorPIR(pirPin);
void onPIRDetection(boolean isMotionDeteted) {
    FRAME frame;
    SewParser::encodeToggle(frame, macPIR, sensorPIR.isMotionDetected());
    sewMQTT.publish(0, frame.frame, frame.size);
}

// DCMotor sensors
SewMotors leftMotor(enableM1, forwardM1, reverseM1);
SewMotors rightMotor(enabledM2, forwardM2, reverseM2);

// MQTT
// Subscribers
String MAC = WiFi.macAddress();
String subscriptionActions = MAC + "/action";
String subscriptions[] = {subscriptionActions};

// Publishers
String publishData = MAC + "/status";
String publications[] = {publishData};

// Sew Parser callback
SewParser sewParser;
uint8_t sewParserBuffer[SEW_PARSER_MAX_BUFFER_SIZE];
int handleFrames(FRAME frame, int status) {
    if(frame.type == DCMOTOR) {
        uint8_t data[3];
        decodeDCMotorPayload(frame, data);
        if(frame.mac[7] == 0x01) {
            leftMotor.runMotor(data[0], data[1], data[2]);
        }
        else {
            rightMotor.runMotor(data[0], data[1], data[2]);
        }
    }
    return 0;
}

void mqttMessages(char* topic, uint8_t* payload, uint8_t length)
{
    sewParser.decodeFrameWithCallback(payload, length, handleFrames);
}

void onMQTTConnect() {
    FRAME frame;
    SewParser::encodeDCMotor(frame, macMotorLeft, 0, 0, 0);
    sewMQTT.publish(0, frame.frame, frame.size);
    SewParser::encodeDCMotor(frame, macMotorRight, 0, 0, 0);
    sewMQTT.publish(0, frame.frame, frame.size);
}

void onSensorRequest(WebServer& ws) {
    String response = "{";
    response += "\"deviceId\": \"" + MAC + "\",";
    response += "\"sensors\": [";
    response += "{\"type\": \"DCMOTOR\", \"sensorId\": \"" + MAC + ":00:01\"},";
    response += "{\"type\": \"DCMOTOR\", \"sensorId\": \"" + MAC + ":00:02\"},";
    response += "{\"type\": \"DISTANCE\", \"sensorId\": \"" + MAC + ":00:03\"},";
    response += "{\"type\": \"DISTANCE\", \"sensorId\": \"" + MAC + ":00:04\"},";
    response += "{\"type\": \"TOGGLE\", \"sensorId\": \"" + MAC + ":00:05\"},";
    response += "]}";
    ws.send(200, "application/json", response);
}

void setup()
{
    // Debug
    Serial.begin(115200);
    // Try to connect to last WiFi
    initWifi();
    // Init WebServer + mqtt client
    sewServer.addRequest("/sensors", onSensorRequest);
    sewServer.initServer(&sewMQTT);
    sewMQTT.setSubscribers(subscriptions, 1);
    sewMQTT.setPublishers(publications, 1);
    sewMQTT.initMQTT(mqttMessages, onMQTTConnect);
    sewMQTT.reconnect();

    // Setup sensors
    distanceFront.init();
    distanceFront.registerCallback(1000, distanceFrontCallback);
    distanceBack.init();
    distanceBack.registerCallback(1000, distanceBackCallback);

    sensorPIR.init();
    sensorPIR.registerCallback(onPIRDetection);

    leftMotor.init();
    rightMotor.init();
}

void loop()
{
    // Only for TESTING
    // int byteReaded = 0;
    // while(Serial.available() > 0 && byteReaded < SEW_PARSER_MAX_BUFFER_SIZE) {
    //     sewParserBuffer[byteReaded] = Serial.read();
    //     byteReaded++;
    // }
    // if(byteReaded > 0) {
    //     sewParser.decodeFrameWithCallback(sewParserBuffer, byteReaded, handleFrames);
    // }

    sewServer.handleSewWebServer();
    sewMQTT.handleClient();
    distanceFront.handleDistances();
    distanceBack.handleDistances();
    sensorPIR.handlePIR();
}