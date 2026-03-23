#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "config.h"

using SetpointCallback = void (*)(int circuitIndex, float setpoint);

/**
 * MQTT client for publishing sensor data and receiving setpoint changes.
 *
 * Topic structure:
 *   {prefix}/circuit/{n}/temperature   → publish
 *   {prefix}/circuit/{n}/setpoint      → subscribe (receive)
 *   {prefix}/circuit/{n}/valve_position → publish
 *   {prefix}/outdoor/temperature       → publish
 *   {prefix}/status                    → publish (LWT)
 *   {prefix}/config                    → subscribe (receive)
 */
class MqttClient {
public:
    void begin(const Config& config, SetpointCallback callback) {
        _prefix = config.mqttPrefix;
        _circuitCount = config.circuitCount;
        _setpointCallback = callback;

        _wifiClient = new WiFiClient();
        _client = new PubSubClient(*_wifiClient);
        _client->setServer(config.mqttBroker.c_str(), config.mqttPort);
        _client->setCallback([this](char* topic, byte* payload, unsigned int length) {
            this->onMessage(topic, payload, length);
        });
        _client->setBufferSize(1024);

        connect();
    }

    void loop() {
        if (!_client->connected()) {
            unsigned long now = millis();
            if (now - _lastReconnectAttempt > 5000) {
                _lastReconnectAttempt = now;
                connect();
            }
        }
        _client->loop();
    }

    void publishTemperature(int circuit, float temp) {
        if (temp < -100.0f) return;  // Skip invalid readings
        String topic = _prefix + "/circuit/" + String(circuit) + "/temperature";
        _client->publish(topic.c_str(), String(temp, 2).c_str(), true);
    }

    void publishSetpoint(int circuit, float setpoint) {
        String topic = _prefix + "/circuit/" + String(circuit) + "/setpoint";
        _client->publish(topic.c_str(), String(setpoint, 1).c_str(), true);
    }

    void publishValvePosition(int circuit, float position) {
        String topic = _prefix + "/circuit/" + String(circuit) + "/valve_position";
        _client->publish(topic.c_str(), String(position, 1).c_str(), true);
    }

    void publishOutdoorTemperature(float temp) {
        if (temp < -100.0f) return;
        String topic = _prefix + "/outdoor/temperature";
        _client->publish(topic.c_str(), String(temp, 2).c_str(), true);
    }

    void publishStatus(const char* status) {
        String topic = _prefix + "/status";
        _client->publish(topic.c_str(), status, true);
    }

    bool isConnected() const {
        return _client && _client->connected();
    }

private:
    void connect() {
        if (!_client) return;

        String clientId = "open-heating-" + String(random(0xFFFF), HEX);
        String willTopic = _prefix + "/status";

        if (_client->connect(clientId.c_str(),
                             nullptr, nullptr,  // no auth (configurable later)
                             willTopic.c_str(), 1, true, "offline")) {
            Serial.println("[MQTT] Connected");

            // Subscribe to setpoint topics for all circuits
            for (int i = 0; i < _circuitCount; i++) {
                String topic = _prefix + "/circuit/" + String(i) + "/setpoint/set";
                _client->subscribe(topic.c_str());
            }

            // Subscribe to config topic
            String configTopic = _prefix + "/config/set";
            _client->subscribe(configTopic.c_str());

            publishStatus("online");
        } else {
            Serial.printf("[MQTT] Connection failed: %d\n", _client->state());
        }
    }

    void onMessage(char* topic, byte* payload, unsigned int length) {
        String topicStr(topic);
        String payloadStr;
        payloadStr.reserve(length);
        for (unsigned int i = 0; i < length; i++) {
            payloadStr += (char)payload[i];
        }

        Serial.printf("[MQTT] Received: %s = %s\n",
                      topic, payloadStr.c_str());

        // Parse setpoint messages: {prefix}/circuit/{n}/setpoint/set
        for (int i = 0; i < _circuitCount; i++) {
            String expected = _prefix + "/circuit/" + String(i) + "/setpoint/set";
            if (topicStr == expected) {
                float setpoint = payloadStr.toFloat();
                if (setpoint >= 10.0f && setpoint <= 80.0f && _setpointCallback) {
                    _setpointCallback(i, setpoint);
                }
                return;
            }
        }

        // TODO: handle config updates
    }

    WiFiClient* _wifiClient = nullptr;
    PubSubClient* _client = nullptr;
    String _prefix;
    int _circuitCount = 0;
    SetpointCallback _setpointCallback = nullptr;
    unsigned long _lastReconnectAttempt = 0;
};
