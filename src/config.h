#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>

enum class SensorType {
    DS18B20,
    PT1000
};

struct HeatingCurveParams;  // Forward declaration, defined in heating_curve.h

struct CircuitConfig {
    String name = "Circuit";
    SensorType sensorType = SensorType::DS18B20;
    uint8_t ds18b20Address[8] = {};
    uint8_t pt1000CsPin = 5;
    uint8_t pinOpen = 0;
    uint8_t pinClose = 0;
    float runTimeSeconds = 60.0f;
    float kp = 2.0f;
    float ki = 0.1f;
    float kd = 0.5f;
    bool heatingCurveEnabled = true;

    // Heating curve (inline to avoid circular dependency)
    float hcOutdoorMin = -15.0f;
    float hcOutdoorMax = 20.0f;
    float hcSupplyAtMin = 55.0f;
    float hcSupplyAtMax = 25.0f;
};

struct Config {
    // WiFi
    String wifiSsid;
    String wifiPassword;

    // MQTT
    String mqttBroker = "192.168.1.100";
    int mqttPort = 1883;
    String mqttPrefix = "open-heating";

    // Outdoor sensor
    bool hasOutdoorSensor = true;
    uint8_t outdoorSensorAddress[8] = {};

    // Circuits
    int circuitCount = 0;
    CircuitConfig circuits[8];

    bool load(const char* path) {
        File file = SPIFFS.open(path, "r");
        if (!file) {
            Serial.printf("[Config] Failed to open %s\n", path);
            return false;
        }

        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, file);
        file.close();

        if (error) {
            Serial.printf("[Config] Parse error: %s\n", error.c_str());
            return false;
        }

        // WiFi
        wifiSsid = doc["wifi"]["ssid"] | "";
        wifiPassword = doc["wifi"]["password"] | "";

        // MQTT
        mqttBroker = doc["mqtt"]["broker"] | "192.168.1.100";
        mqttPort = doc["mqtt"]["port"] | 1883;
        mqttPrefix = doc["mqtt"]["prefix"] | "open-heating";

        // Outdoor sensor
        if (doc["outdoor_sensor"].is<JsonObject>()) {
            hasOutdoorSensor = true;
            parseAddress(doc["outdoor_sensor"]["address"] | "",
                        outdoorSensorAddress);
        }

        // Circuits
        JsonArray circuitsArr = doc["circuits"].as<JsonArray>();
        circuitCount = min((int)circuitsArr.size(), 8);

        for (int i = 0; i < circuitCount; i++) {
            JsonObject c = circuitsArr[i];
            circuits[i].name = c["name"] | ("Circuit " + String(i));

            // Sensor
            String sType = c["sensor"]["type"] | "ds18b20";
            if (sType == "pt1000") {
                circuits[i].sensorType = SensorType::PT1000;
                circuits[i].pt1000CsPin = c["sensor"]["cs_pin"] | 5;
            } else {
                circuits[i].sensorType = SensorType::DS18B20;
                parseAddress(c["sensor"]["address"] | "",
                           circuits[i].ds18b20Address);
            }

            // Actuator
            circuits[i].pinOpen = c["actuator"]["pin_open"] | 0;
            circuits[i].pinClose = c["actuator"]["pin_close"] | 0;
            circuits[i].runTimeSeconds = c["actuator"]["run_time_seconds"] | 60.0f;

            // PID
            circuits[i].kp = c["pid"]["kp"] | 2.0f;
            circuits[i].ki = c["pid"]["ki"] | 0.1f;
            circuits[i].kd = c["pid"]["kd"] | 0.5f;

            // Heating curve
            if (c["heating_curve"].is<JsonObject>()) {
                circuits[i].heatingCurveEnabled = true;
                circuits[i].hcOutdoorMin = c["heating_curve"]["outdoor_min"] | -15.0f;
                circuits[i].hcOutdoorMax = c["heating_curve"]["outdoor_max"] | 20.0f;
                circuits[i].hcSupplyAtMin = c["heating_curve"]["supply_at_min"] | 55.0f;
                circuits[i].hcSupplyAtMax = c["heating_curve"]["supply_at_max"] | 25.0f;
            }
        }

        Serial.printf("[Config] Loaded: %d circuits, MQTT %s:%d\n",
                      circuitCount, mqttBroker.c_str(), mqttPort);
        return true;
    }

    bool save(const char* path) {
        JsonDocument doc;

        doc["wifi"]["ssid"] = wifiSsid;
        doc["wifi"]["password"] = wifiPassword;
        doc["mqtt"]["broker"] = mqttBroker;
        doc["mqtt"]["port"] = mqttPort;
        doc["mqtt"]["prefix"] = mqttPrefix;

        // TODO: serialize circuits back to JSON

        File file = SPIFFS.open(path, "w");
        if (!file) return false;
        serializeJsonPretty(doc, file);
        file.close();
        return true;
    }

private:
    void parseAddress(const String& addrStr, uint8_t* out) {
        // Parse "28-01234567abcd" format
        if (addrStr.length() < 2) return;
        String clean = addrStr;
        clean.replace("-", "");
        for (int i = 0; i < 8 && i * 2 < (int)clean.length(); i++) {
            String byteStr = clean.substring(i * 2, i * 2 + 2);
            out[i] = strtol(byteStr.c_str(), nullptr, 16);
        }
    }
};
