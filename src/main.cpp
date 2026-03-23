#include <Arduino.h>
#include <WiFi.h>
#include <SPIFFS.h>

#include "config.h"
#include "sensor_manager.h"
#include "actuator_driver.h"
#include "control_loop.h"
#include "heating_curve.h"
#include "mqtt_client.h"
#include "ota_updater.h"

// Maximum number of heating circuits
static constexpr int MAX_CIRCUITS = 8;

// Watchdog timeout
static constexpr int WDT_TIMEOUT_S = 10;

// Control loop interval
static constexpr unsigned long CONTROL_INTERVAL_MS = 5000;

Config config;
SensorManager sensors;
ActuatorDriver actuators[MAX_CIRCUITS];
ControlLoop controllers[MAX_CIRCUITS];
HeatingCurve heatingCurve;
MqttClient mqtt;
OtaUpdater ota;

int activeCircuits = 0;
unsigned long lastControlRun = 0;

void setupWiFi() {
    Serial.printf("[WiFi] Connecting to %s", config.wifiSsid.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.setHostname("open-heating");
    WiFi.begin(config.wifiSsid.c_str(), config.wifiPassword.c_str());

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WiFi] Connected: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\n[WiFi] Connection failed — running in offline mode");
    }
}

void onSetpointReceived(int circuitIndex, float setpoint) {
    if (circuitIndex >= 0 && circuitIndex < activeCircuits) {
        controllers[circuitIndex].setSetpoint(setpoint);
        Serial.printf("[MQTT] Circuit %d setpoint: %.1f°C\n", circuitIndex, setpoint);
    }
}

void runControlLoop() {
    // Read outdoor temperature for heating curve
    float outdoorTemp = sensors.readOutdoorTemperature();

    for (int i = 0; i < activeCircuits; i++) {
        // Read circuit temperature
        float currentTemp = sensors.readCircuitTemperature(i);

        // Calculate weather-compensated setpoint if enabled
        float setpoint = controllers[i].getSetpoint();
        if (config.circuits[i].heatingCurveEnabled) {
            setpoint = heatingCurve.calculate(
                outdoorTemp,
                config.circuits[i].heatingCurveParams
            );
            controllers[i].setSetpoint(setpoint);
        }

        // Run PID controller → get valve target position (0–100%)
        float valveTarget = controllers[i].update(currentTemp);

        // Drive actuator toward target position
        actuators[i].moveTo(valveTarget);

        // Publish telemetry
        mqtt.publishTemperature(i, currentTemp);
        mqtt.publishSetpoint(i, setpoint);
        mqtt.publishValvePosition(i, actuators[i].getEstimatedPosition());
    }

    mqtt.publishOutdoorTemperature(outdoorTemp);
    mqtt.publishStatus("online");
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n=============================");
    Serial.println("  open-heating firmware v0.1");
    Serial.println("=============================\n");

    // Load configuration from SPIFFS
    if (!SPIFFS.begin(true)) {
        Serial.println("[SPIFFS] Mount failed!");
        return;
    }
    config.load("/config.json");
    activeCircuits = config.circuitCount;
    Serial.printf("[Config] %d circuits configured\n", activeCircuits);

    // Initialize sensors
    sensors.begin(config);

    // Initialize actuators and run reference (fully close to establish 0%)
    Serial.println("[Actuator] Starting reference run...");
    for (int i = 0; i < activeCircuits; i++) {
        actuators[i].begin(
            config.circuits[i].pinOpen,
            config.circuits[i].pinClose,
            config.circuits[i].runTimeSeconds
        );
        actuators[i].referenceRun();
    }
    Serial.println("[Actuator] Reference run complete");

    // Initialize heating curve
    heatingCurve.begin();

    // Network
    setupWiFi();
    mqtt.begin(config, onSetpointReceived);
    ota.begin();

    // Enable watchdog
    esp_task_wdt_init(WDT_TIMEOUT_S, true);
    esp_task_wdt_add(NULL);

    Serial.println("[Setup] Complete — entering control loop\n");
}

void loop() {
    // Feed watchdog
    esp_task_wdt_reset();

    // Handle OTA updates
    ota.handle();

    // Handle MQTT
    mqtt.loop();

    // Run control loop at fixed interval
    unsigned long now = millis();
    if (now - lastControlRun >= CONTROL_INTERVAL_MS) {
        lastControlRun = now;
        runControlLoop();
    }

    // Update actuator state machines (handles timed relay switching)
    for (int i = 0; i < activeCircuits; i++) {
        actuators[i].update();
    }
}
