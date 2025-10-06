#include "servo_motor.hpp"

ServoMotor::ServoMotor(uint8_t controlPin, uint16_t minPulseWidth, uint16_t maxPulseWidth)
    : _pin(controlPin),
      _minPulseWidth(minPulseWidth),
      _maxPulseWidth(maxPulseWidth),
      _attached(false),
      _currentDegrees(0.0f),
      _targetDegrees(0.0f),
      _speedDegreesPerSecond(0.0f),
      _lastUpdateTime(0) {}

void ServoMotor::setup() {
    if (!_attached) {
        _servo.attach(_pin, _minPulseWidth, _maxPulseWidth);
        _attached = true;
        _servo.write(static_cast<int>(_currentDegrees));
        _lastUpdateTime = millis();
    }
}

void ServoMotor::writeAngle(float degrees) {
    if (!_attached) {
        setup();
    }

    _currentDegrees = constrain(degrees, 0.0f, 180.0f);
    _targetDegrees = _currentDegrees;
    _servo.write(static_cast<int>(_currentDegrees));
}

void ServoMotor::writeAngleWithSpeed(float degrees, float degreesPerSecond) {
    if (!_attached) {
        setup();
    }

    _targetDegrees = constrain(degrees, 0.0f, 180.0f);
    _speedDegreesPerSecond = degreesPerSecond;
}

void ServoMotor::update() {
    if (!_attached || _speedDegreesPerSecond <= 0.0f) {
        return;
    }

    unsigned long currentTime = millis();
    float dt = (currentTime - _lastUpdateTime) / 1000.0f; // Convert to seconds
    _lastUpdateTime = currentTime;

    if (dt > 0.0f) {
        float maxMovement = _speedDegreesPerSecond * dt;
        float error = _targetDegrees - _currentDegrees;

        if (fabs(error) <= maxMovement) {
            // Close enough to target
            _currentDegrees = _targetDegrees;
            _speedDegreesPerSecond = 0.0f; // Stop moving
        } else {
            // Move towards target
            _currentDegrees += copysign(maxMovement, error);
        }

        _servo.write(static_cast<int>(_currentDegrees));
    }
}

void ServoMotor::detach() {
    if (_attached) {
        _servo.detach();
        _attached = false;
    }
}