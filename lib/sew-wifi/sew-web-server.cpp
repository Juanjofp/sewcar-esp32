#include "sew-web-server.hpp"

SewWebServer::SewWebServer () : server(80), espClient()
{
}

DynamicJsonDocument SewWebServer::parseBody(int size)
{
    const size_t capacity = JSON_OBJECT_SIZE(3) + size;
    DynamicJsonDocument doc(capacity);
    deserializeJson(doc, server.arg("plain"));
    return doc;
}

void SewWebServer::handleModeAp()
{
    WiFiMode_t mode = WiFi.getMode();
    if (mode != WIFI_AP)
    {
        if (server.method() != HTTP_POST)
        {
            server.send(200, "application/json", getConfiguration());
        }
        else
        {
            server.send(200, "application/json", getConfigurationAP());
            delay(500);
            enableWifiAP();
        }
    }
    else
    {
        server.send(200, "application/json", getConfiguration());
    }
}

void SewWebServer::handleModeSta()
{
    WiFiMode_t mode = WiFi.getMode();
    if (mode != WIFI_STA)
    {
        if (server.method() != HTTP_POST)
        {
            server.send(200, "application/json", getConfiguration());
        }
        else
        {
            DynamicJsonDocument doc = parseBody(60);
            String ssid = doc["ssid"];  // "CenticWifi"
            String passwd = doc["pwd"]; // "123456789"
            if (ssid.length() > 3)
            {
                // Try to connect firsst!
                bool tryNewConnection = tryWifiClient(ssid.c_str(), passwd.c_str());
                // delay(3000);
                if (tryNewConnection)
                {
                    server.send(200, "application/json", getConfiguration());
                    delay(3000);
                    enableWifiClient(ssid.c_str(), passwd.c_str());
                }
                else
                {
                    server.send(200, "application/json", getConfigurationAP());
                }
            }
            else
            {
                server.send(400, "application/json", "{\"code\": 404, \"error\": \"Invalid SSID, min 3 chars\"}");
            }
        }
    }
    else
    {
        server.send(200, "application/json", getConfiguration());
    }
}

void SewWebServer::handleInfo()
{
    server.send(200, "application/json", getConfiguration());
}

void SewWebServer::handleMQTT()
{
    WiFiMode_t mode = WiFi.getMode();
    if (mode != WIFI_AP)
    {
        if (server.method() != HTTP_POST)
        {
            server.send(200, "application/json", mqtt->getInfo());
        }
        else
        {
            DynamicJsonDocument doc = parseBody(256);
            String host = doc["host"];   // "sew.tribeca.ovh"
            int port = doc["port"];      // 1885
            String topic = doc["topic"]; // juanjo/casa1

            mqtt->disconnect();
            mqtt->setConfig(host, port, topic);
            bool connected = mqtt->reconnect();
            if (connected)
            {
                server.send(200, "application/json", mqtt->getInfo());
            }
            else
            {
                server.send(400, "application/json", "{\"code\": 400, \"details\":\"Cannot connect to MQTT\"}");
            }
        }
    }
    else
    {
        server.send(200, "application/json", getConfiguration());
    }
}

void SewWebServer::handle404()
{
    server.send(404, "application/json", "{\"code\": 404, \"error\": \"Endpoint not found\"}");
}

void SewWebServer::addRequest(const String &uri, void(*request)(WebServer& server))
{
    server.on(uri, [this, request](){request(this->server);});
}

void SewWebServer::initServer(SewMQTT* sewMQTT)
{
    this->mqtt = sewMQTT;
    // Rutas por defecto
    server.on("/mode/ap", [this](){this->handleModeAp();});
    server.on("/mode/sta", [this](){this->handleModeSta();});
    server.on("/info", [this](){this->handleInfo();});
    server.on("/mqtt", [this](){this->handleMQTT();});
    server.onNotFound([this](){this->handle404();});

    // Iniciar servidor
    server.begin();
    delay(1500);
}

void SewWebServer::handleSewWebServer()
{
    server.handleClient();
}
