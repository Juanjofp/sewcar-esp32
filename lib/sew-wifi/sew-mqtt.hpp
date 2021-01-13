#ifndef MQTT_SEW_HPP
#define MQTT_SEW_HPP
#include <WiFi.h>
#include <PubSubClient.h>

class SewMQTT
{
    public:
        SewMQTT();
        void initMQTT(MQTT_CALLBACK_SIGNATURE, void (*onConnect)());
        void setConfig(String host, int port, String topic);
        void setSubscribers(String subscribers[], size_t len);
        void setPublishers(String publishers[], size_t len);
        int publish(int channel, const uint8_t* payload, unsigned int length);
        String getInfo();
        bool reconnect();
        bool disconnect();
        void handleClient();

    private:
        WiFiClient espClient;
        PubSubClient *client = NULL;
        void (*onConnect)();
        String host = "sew.ovh";
        int port = 30810;
        String topic = "tribeca/";
        String *subscribers;
        size_t totalSubscribers;
        String *publishers;
        size_t totalPublishers;
        long lastReconnectAttempt = 0;
        void (*callback)(String topic, String payload, unsigned int length);
        void readConfig();
        void saveConfig();
        boolean isConfigChanged = false;
};
#endif
