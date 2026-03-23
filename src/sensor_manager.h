#pragma once

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_MAX31865.h>
#include "config.h"

/**
 * Abstracts temperature sensor access for both DS18B20 (1-Wire)
 * and PT1000 (via MAX31865 SPI breakout).
 *
 * DS18B20: Multiple sensors on a single 1-Wire bus, addressed individually.
 * PT1000:  Each sensor needs its own MAX31865 + dedicated CS pin on SPI.
 */
class SensorManager {
public:
    static constexpr int MAX_CIRCUITS = 8;
    static constexpr float INVALID_TEMP = -127.0f;

    // DS18B20 1-Wire bus pin
    static constexpr uint8_t ONEWIRE_PIN = 4;

    // SPI pins for MAX31865 (PT1000)
    static constexpr uint8_t SPI_MOSI = 23;
    static constexpr uint8_t SPI_MISO = 19;
    static constexpr uint8_t SPI_CLK  = 18;

    void begin(const Config& config) {
        _circuitCount = config.circuitCount;

        // Initialize 1-Wire bus for DS18B20 sensors
        _oneWire = new OneWire(ONEWIRE_PIN);
        _dallas = new DallasTemperature(_oneWire);
        _dallas->begin();
        _dallas->setResolution(12);  // 12-bit = 0.0625°C, ~750ms conversion
        _dallas->setWaitForConversion(false);  // Async reads

        // Initialize PT1000 sensors (one MAX31865 per sensor)
        for (int i = 0; i < _circuitCount; i++) {
            _sensorTypes[i] = config.circuits[i].sensorType;

            if (config.circuits[i].sensorType == SensorType::PT1000) {
                _max31865[i] = new Adafruit_MAX31865(
                    config.circuits[i].pt1000CsPin,
                    SPI_MOSI, SPI_MISO, SPI_CLK
                );
                // PT1000: use 4-wire config, reference resistor = 4300 Ohm
                _max31865[i]->begin(MAX31865_4WIRE);
                Serial.printf("[Sensor] Circuit %d: PT1000 on CS pin %d\n",
                              i, config.circuits[i].pt1000CsPin);
            } else {
                // Store DS18B20 address for this circuit
                memcpy(_ds18b20Addr[i],
                       config.circuits[i].ds18b20Address, 8);
                Serial.printf("[Sensor] Circuit %d: DS18B20\n", i);
            }
        }

        // Outdoor sensor is always DS18B20
        _hasOutdoorSensor = config.hasOutdoorSensor;
        if (_hasOutdoorSensor) {
            memcpy(_outdoorAddr, config.outdoorSensorAddress, 8);
            Serial.println("[Sensor] Outdoor: DS18B20");
        }

        // Kick off first async conversion
        _dallas->requestTemperatures();
        _lastConversionRequest = millis();

        Serial.printf("[Sensor] Initialized %d circuit sensors\n", _circuitCount);
    }

    /**
     * Read temperature for a heating circuit.
     * Returns INVALID_TEMP (-127°C) on read failure.
     */
    float readCircuitTemperature(int circuitIndex) {
        if (circuitIndex < 0 || circuitIndex >= _circuitCount) {
            return INVALID_TEMP;
        }

        if (_sensorTypes[circuitIndex] == SensorType::PT1000) {
            return readPT1000(circuitIndex);
        } else {
            return readDS18B20(_ds18b20Addr[circuitIndex]);
        }
    }

    /**
     * Read outdoor temperature (always DS18B20).
     */
    float readOutdoorTemperature() {
        if (!_hasOutdoorSensor) return INVALID_TEMP;
        return readDS18B20(_outdoorAddr);
    }

    /**
     * Call periodically to manage async DS18B20 conversions.
     */
    void requestConversions() {
        unsigned long now = millis();
        if (now - _lastConversionRequest >= 1000) {
            _dallas->requestTemperatures();
            _lastConversionRequest = now;
        }
    }

private:
    float readDS18B20(const uint8_t addr[8]) {
        float temp = _dallas->getTempC(addr);
        if (temp == DEVICE_DISCONNECTED_C) {
            return INVALID_TEMP;
        }
        return temp;
    }

    float readPT1000(int index) {
        if (!_max31865[index]) return INVALID_TEMP;

        // PT1000: nominal resistance = 1000 Ohm, ref resistor = 4300 Ohm
        float temp = _max31865[index]->temperature(1000.0f, 4300.0f);

        // Check for faults
        uint8_t fault = _max31865[index]->readFault();
        if (fault) {
            Serial.printf("[Sensor] PT1000 circuit %d fault: 0x%02X\n",
                          index, fault);
            _max31865[index]->clearFault();
            return INVALID_TEMP;
        }

        return temp;
    }

    OneWire* _oneWire = nullptr;
    DallasTemperature* _dallas = nullptr;
    Adafruit_MAX31865* _max31865[MAX_CIRCUITS] = {};

    SensorType _sensorTypes[MAX_CIRCUITS] = {};
    uint8_t _ds18b20Addr[MAX_CIRCUITS][8] = {};
    uint8_t _outdoorAddr[8] = {};

    int _circuitCount = 0;
    bool _hasOutdoorSensor = false;
    unsigned long _lastConversionRequest = 0;
};
