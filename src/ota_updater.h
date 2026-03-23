#pragma once

#include <Arduino.h>
#include <ArduinoOTA.h>

/**
 * Over-the-Air update handler.
 * Allows firmware updates via WiFi without physical USB access.
 *
 * Usage:
 *   pio run --target upload --upload-port <esp32-ip>
 */
class OtaUpdater {
public:
    void begin() {
        ArduinoOTA.setHostname("open-heating");

        ArduinoOTA.onStart([]() {
            String type = (ArduinoOTA.getCommand() == U_FLASH)
                        ? "firmware" : "filesystem";
            Serial.printf("[OTA] Start updating %s\n", type.c_str());
        });

        ArduinoOTA.onEnd([]() {
            Serial.println("\n[OTA] Update complete — restarting");
        });

        ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("[OTA] Progress: %u%%\r", (progress * 100) / total);
        });

        ArduinoOTA.onError([](ota_error_t error) {
            Serial.printf("[OTA] Error[%u]: ", error);
            switch (error) {
                case OTA_AUTH_ERROR:    Serial.println("Auth Failed"); break;
                case OTA_BEGIN_ERROR:   Serial.println("Begin Failed"); break;
                case OTA_CONNECT_ERROR: Serial.println("Connect Failed"); break;
                case OTA_RECEIVE_ERROR: Serial.println("Receive Failed"); break;
                case OTA_END_ERROR:     Serial.println("End Failed"); break;
            }
        });

        ArduinoOTA.begin();
        Serial.println("[OTA] Ready");
    }

    void handle() {
        ArduinoOTA.handle();
    }
};
