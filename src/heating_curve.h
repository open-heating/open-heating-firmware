#pragma once

#include <Arduino.h>

/**
 * Heating curve parameters for weather-compensated control.
 *
 * Maps outdoor temperature to supply (Vorlauf) temperature setpoint.
 * Uses linear interpolation between two anchor points:
 *
 *   outdoor_min (e.g., -15°C) → supply_at_min (e.g., 55°C)
 *   outdoor_max (e.g., +20°C) → supply_at_max (e.g., 25°C)
 *
 * Below outdoor_min: clamps to supply_at_min
 * Above outdoor_max: heating off (returns 0 or supply_at_max)
 *
 * The curve can be shifted up/down by a room offset:
 *   If room temp is below room setpoint → shift curve up (more heat)
 *   If room temp is above room setpoint → shift curve down (less heat)
 */
struct HeatingCurveParams {
    float outdoorMin = -15.0f;    // Design outdoor temp (°C)
    float outdoorMax = 20.0f;     // Heating shutoff outdoor temp (°C)
    float supplyAtMin = 55.0f;    // Supply temp at coldest outdoor (°C)
    float supplyAtMax = 25.0f;    // Supply temp at warmest outdoor (°C)
    float roomSetpoint = 21.0f;   // Target room temperature (°C)
    float roomInfluence = 3.0f;   // °C supply shift per °C room deviation
    float minSupplyTemp = 20.0f;  // Absolute minimum supply temp (°C)
    float maxSupplyTemp = 65.0f;  // Absolute maximum supply temp (°C)
};

class HeatingCurve {
public:
    void begin() {
        Serial.println("[HeatingCurve] Initialized");
    }

    /**
     * Calculate supply temperature setpoint from outdoor temperature.
     *
     * @param outdoorTemp  Current outdoor temperature (°C)
     * @param params       Heating curve parameters for this circuit
     * @param roomTemp     Optional: current room temperature for feedback
     *                     Pass NAN if no room sensor available
     * @return Supply temperature setpoint (°C)
     */
    float calculate(float outdoorTemp,
                    const HeatingCurveParams& params,
                    float roomTemp = NAN) {

        // Guard against invalid readings
        if (outdoorTemp < -50.0f || outdoorTemp > 50.0f) {
            return params.supplyAtMin;  // Fail-safe: heat at max
        }

        // Linear interpolation
        float t = (outdoorTemp - params.outdoorMin)
                / (params.outdoorMax - params.outdoorMin);
        t = constrain(t, 0.0f, 1.0f);

        float supplyTemp = params.supplyAtMin
                         + t * (params.supplyAtMax - params.supplyAtMin);

        // Room temperature feedback (if available)
        if (!isnan(roomTemp)) {
            float roomError = params.roomSetpoint - roomTemp;
            supplyTemp += roomError * params.roomInfluence;
        }

        // Clamp to safe range
        supplyTemp = constrain(supplyTemp,
                               params.minSupplyTemp,
                               params.maxSupplyTemp);

        return supplyTemp;
    }

    /**
     * Check if heating should be active based on outdoor temperature.
     * Returns false if outdoor temp is above the shutoff threshold.
     */
    bool isHeatingRequired(float outdoorTemp,
                           const HeatingCurveParams& params) {
        return outdoorTemp < params.outdoorMax;
    }
};
