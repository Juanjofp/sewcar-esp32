#ifndef SEW_WEB_SERVER_HPP
#define SEW_WEB_SERVER_HPP
#include <ArduinoJson.h>
#include <WebServer.h>
#include "sew-wifi-config.h"
#include "sew-mqtt.hpp"

class SewWebServer
{
private:
    WebServer server;
    WiFiClient espClient;
    SewMQTT* mqtt;
public:
    SewWebServer();
    DynamicJsonDocument parseBody(int size);
    void handleModeAp();
    void handleModeSta();
    void handleInfo();
    void handleMQTT();
    void handle404();
    void addRequest(const String &uri, void(*request)(WebServer& server));
    void initServer(SewMQTT* sewMQTT);
    void handleSewWebServer();
};
#endif
