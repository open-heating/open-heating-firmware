#pragma once

#include <Arduino.h>

/**
 * Driver for 3-point actuators (e.g., ESBE ARA600 series).
 *
 * 3-point control: two relays per actuator.
 *   - Relay OPEN  → motor drives valve toward open (CW)
 *   - Relay CLOSE → motor drives valve toward closed (CCW)
 *   - Both OFF    → motor stops, valve holds position
 *
 * SAFETY: OPEN and CLOSE are NEVER energized simultaneously.
 *
 * Position estimation: since 3-point actuators have no position feedback,
 * we estimate position by tracking cumulative open/close time relative
 * to the full run time (e.g., 60s for ARA661).
 *
 * Reference run: on startup, drives fully closed for 1.1x run time
 * to establish a known 0% baseline.
 */
class ActuatorDriver {
public:
    enum class State {
        IDLE,
        OPENING,
        CLOSING,
        REFERENCE_RUN
    };

    void begin(uint8_t pinOpen, uint8_t pinClose, float runTimeSeconds) {
        _pinOpen = pinOpen;
        _pinClose = pinClose;
        _runTimeMs = static_cast<unsigned long>(runTimeSeconds * 1000);

        pinMode(_pinOpen, OUTPUT);
        pinMode(_pinClose, OUTPUT);
        stop();  // Ensure safe state
    }

    /**
     * Perform reference run: close fully to establish 0% position.
     * Blocks for the duration of the run (run_time * 1.1).
     */
    void referenceRun() {
        _state = State::REFERENCE_RUN;
        unsigned long refTime = static_cast<unsigned long>(_runTimeMs * 1.1f);

        stop();
        delay(100);  // Brief pause before direction change

        driveClose();
        delay(refTime);
        stop();

        _estimatedPosition = 0.0f;
        _state = State::IDLE;

        Serial.printf("[Actuator] Pin %d/%d: reference complete (0%%)\n",
                      _pinOpen, _pinClose);
    }

    /**
     * Set target position (0–100%).
     * The update() method will drive the actuator toward this target.
     */
    void moveTo(float targetPercent) {
        _targetPosition = constrain(targetPercent, 0.0f, 100.0f);
    }

    /**
     * Call this frequently (every loop iteration).
     * Manages timed relay switching to reach target position.
     */
    void update() {
        if (_state == State::REFERENCE_RUN) return;

        float error = _targetPosition - _estimatedPosition;
        float deadband = 2.0f;  // Don't hunt within ±2%

        if (abs(error) <= deadband) {
            if (_state != State::IDLE) {
                stop();
                _state = State::IDLE;
            }
            return;
        }

        unsigned long now = millis();

        if (error > 0 && _state != State::OPENING) {
            // Need to open more
            stop();
            delay(50);  // Brief dead time between direction changes
            driveOpen();
            _state = State::OPENING;
            _lastMoveStart = now;
        } else if (error < 0 && _state != State::CLOSING) {
            // Need to close more
            stop();
            delay(50);
            driveClose();
            _state = State::CLOSING;
            _lastMoveStart = now;
        }

        // Update position estimate
        if (_state == State::OPENING || _state == State::CLOSING) {
            float elapsedMs = static_cast<float>(now - _lastMoveStart);
            float movementPercent = (elapsedMs / _runTimeMs) * 100.0f;

            if (_state == State::OPENING) {
                _estimatedPosition = constrain(
                    _estimatedPosition + movementPercent, 0.0f, 100.0f);
            } else {
                _estimatedPosition = constrain(
                    _estimatedPosition - movementPercent, 0.0f, 100.0f);
            }
            _lastMoveStart = now;
        }
    }

    float getEstimatedPosition() const {
        return _estimatedPosition;
    }

    State getState() const {
        return _state;
    }

private:
    void driveOpen() {
        digitalWrite(_pinClose, LOW);   // ALWAYS close first!
        delayMicroseconds(100);
        digitalWrite(_pinOpen, HIGH);
    }

    void driveClose() {
        digitalWrite(_pinOpen, LOW);    // ALWAYS open-off first!
        delayMicroseconds(100);
        digitalWrite(_pinClose, HIGH);
    }

    void stop() {
        digitalWrite(_pinOpen, LOW);
        digitalWrite(_pinClose, LOW);
    }

    uint8_t _pinOpen = 0;
    uint8_t _pinClose = 0;
    unsigned long _runTimeMs = 60000;
    float _estimatedPosition = 0.0f;
    float _targetPosition = 0.0f;
    unsigned long _lastMoveStart = 0;
    State _state = State::IDLE;
};
