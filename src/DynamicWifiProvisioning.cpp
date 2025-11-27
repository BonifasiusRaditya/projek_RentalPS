#include "DynamicWifiProvisioning.hpp"

#include <WiFiManager.h>

void provisionWiFi() {
    WiFi.mode(WIFI_STA);

    WiFiManager wiFiManager;

    // When testing wipe credentials.
    wiFiManager.resetSettings();

    if (wiFiManager.autoConnect()) {
        Serial.println("WiFi connected.");
    }
    else {
        Serial.println("WiFi failed to connect!");
        ESP.restart();
    }
}