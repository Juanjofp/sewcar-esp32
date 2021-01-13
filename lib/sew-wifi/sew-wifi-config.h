#ifndef SEW_Wifi_Config_h
#define SEW_Wifi_Config_h
#include <Arduino.h>

void initWifi();
void enableWifiAP();
void enableWifiClient(const char *ssid, const char *password);
bool tryWifiClient(const char *ssid, const char *password);
String getConfiguration();
String getConfigurationAP();
#endif
