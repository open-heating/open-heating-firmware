#pragma once

#include <Arduino.h>

struct PidParams {
    float kp = 2.0f;
    float ki = 0.1f;
    float kd = 0.5f;
    float outputMin = 0.0f;
    float outputMax = 100.0f;
    float integralWindupLimit = 50.0f;
};

/**
 * PID controller that outputs a valve position target (0–100%).
 *
 * The output is consumed by ActuatorDriver, which translates it
 * into timed open/close relay pulses for 3-point actuators.
 *
 * The controller includes:
 * - Anti-windup on the integral term
 * - Derivative-on-measurement (avoids kick on setpoint change)
 * - Configurable output clamping
 */
class ControlLoop {
public:
    void begin(const PidParams& params) {
        _params = params;
        reset();
    }

    void setSetpoint(float setpoint) {
        _setpoint = setpoint;
    }

    float getSetpoint() const {
        return _setpoint;
    }

    /**
     * Run one PID iteration.
     * @param currentTemp Current measured temperature in °C
     * @return Target valve position 0–100%
     */
    float update(float currentTemp) {
        unsigned long now = millis();

        if (_firstRun) {
            _lastTemp = currentTemp;
            _lastTime = now;
            _firstRun = false;
            return _output;
        }

        float dt = (now - _lastTime) / 1000.0f;
        if (dt <= 0.0f) return _output;

        float error = _setpoint - currentTemp;

        // Proportional
        float pTerm = _params.kp * error;

        // Integral with anti-windup
        _integral += error * dt;
        _integral = constrain(_integral,
            -_params.integralWindupLimit,
             _params.integralWindupLimit);
        float iTerm = _params.ki * _integral;

        // Derivative on measurement (not on error — avoids setpoint kick)
        float dMeasurement = (currentTemp - _lastTemp) / dt;
        float dTerm = -_params.kd * dMeasurement;

        // Sum and clamp
        _output = pTerm + iTerm + dTerm;
        _output = constrain(_output, _params.outputMin, _params.outputMax);

        _lastTemp = currentTemp;
        _lastTime = now;

        return _output;
    }

    float getOutput() const {
        return _output;
    }

    void reset() {
        _integral = 0.0f;
        _output = 0.0f;
        _lastTemp = 0.0f;
        _lastTime = 0;
        _firstRun = true;
    }

private:
    PidParams _params;
    float _setpoint = 20.0f;
    float _integral = 0.0f;
    float _output = 0.0f;
    float _lastTemp = 0.0f;
    unsigned long _lastTime = 0;
    bool _firstRun = true;
};
