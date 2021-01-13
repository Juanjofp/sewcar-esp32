#include <EEPROM.h>
#include "sew-mqtt.hpp"
#include <ArduinoJson.h>

DynamicJsonDocument parseString(String body, int size)
{
    const size_t capacity = JSON_OBJECT_SIZE(3) + size;
    DynamicJsonDocument doc(capacity);
    deserializeJson(doc, body);
    return doc;
}

void SewMQTT::readConfig()
{
    String creds;
    EEPROM.begin(512);
    for (int i = 0; i < 512; i++)
    {
        byte ch = EEPROM.read(i);
        creds += (char)ch;
        if (ch == '\0')
        {
            break;
        }
    }

    DynamicJsonDocument doc = parseString(creds, 512);
    String host = doc["host"];
    int port = doc["port"];
    String topic = doc["topic"];
    if (host != "null")
        this->host = host;
    if (port != 0)
        this->port = port;
    if (topic != "null")
        this->topic = topic;
}

void SewMQTT::saveConfig()
{
    String creds = "{\"host\":\"" + this->host + "\", \"port\": " + this->port + ", \"topic\":\"" + this->topic + "\"}";
    EEPROM.begin(512);
    for (int i = 0; i < creds.length(); i++)
    {
        EEPROM.write(i, creds[i]);
    }
    EEPROM.write(creds.length(), '\0');
    EEPROM.commit();
}

SewMQTT::SewMQTT(): espClient()
{
    client = new PubSubClient(espClient);
}

void SewMQTT::initMQTT(MQTT_CALLBACK_SIGNATURE, void (*onConnect)())
{
    readConfig();
    client->setServer(host.c_str(), port);
    client->setCallback(callback);
    this->onConnect = onConnect;
    delay(500);
}

void SewMQTT::handleClient()
{
    if (!client->connected())
    {
        long now = millis();
        if (now - lastReconnectAttempt > 5000)
        {
            lastReconnectAttempt = now;
            // Attempt to reconnect
            if (reconnect())
            {
                lastReconnectAttempt = 0;
            }
        }
    }
    else
    {
        // Client connected
        client->loop();
    }
}

void SewMQTT::setConfig(String host, int port, String topic)
{
    this->host = host;
    this->port = port;
    this->topic = topic;
    isConfigChanged = true;
    client->setServer(this->host.c_str(), this->port);
}

void SewMQTT::setSubscribers(String *subscribers, size_t len) {
    this->subscribers = subscribers;
    this->totalSubscribers = len;
}

void SewMQTT::setPublishers(String *publishers, size_t len) {
    this->publishers = publishers;
    this->totalPublishers = len;
}

int SewMQTT::publish(int channel, const uint8_t* payload, unsigned int plength) {
    if(channel >= 0 && channel < totalPublishers) {
        String targetTopic = this->topic + publishers[channel];
        client->publish(targetTopic.c_str(), payload, plength);
        return 0;
    }
    return -1;
}

String SewMQTT::getInfo()
{
    String response = "{\"host\":\"";
    response += host;
    response += "\",\"port\":";
    response += String(port);
    response += ",\"topic\":\"";
    response += topic;

    response += "\",\"publish\":[";
    for(int i = 0; i < totalPublishers; i++) {
        response += "\"" + topic + publishers[i] + "\"";
        if (i < totalPublishers - 1) {
            response += ",";
        }
    }
    response += "]";

    response += ",\"subscribe\":[";
    for(int i = 0; i < totalSubscribers; i++) {
        response += "\"" + topic + subscribers[i] + "\"";
        if (i < totalSubscribers - 1) {
            response += ",";
        }
    }
    response += "]";

    response += ",\"connected\":";
    response += client->connected() ? "true" : "false";
    response += ",\"clientId\":\"";
    response += WiFi.macAddress().c_str();
    response += "\"}";

    return response;
}

bool SewMQTT::reconnect()
{
    if (client->connect(WiFi.macAddress().c_str()))
    {
        // resuscribimos cada vez q se desconecta
        for(int i = 0; i < totalSubscribers; i++) {
            String fullTopic = topic + subscribers[i];
            client->subscribe(fullTopic.c_str());
        }
        // Cada vez que reconectamos publicamos el estado actual
        this->onConnect();
        if (isConfigChanged)
        {
            isConfigChanged = false;
            saveConfig();
        }
    }
    return client->connected();
}

bool SewMQTT::disconnect()
{
    if (client != NULL)
    {
        client->disconnect();
        return true;
    }
    return false;
}
