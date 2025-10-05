#include "servo_motor.hpp"

ServoMotor::ServoMotor(uint8_t controlPin, uint16_t minPulseWidth, uint16_t maxPulseWidth)
    : _pin(controlPin),
      _minPulseWidth(minPulseWidth),
      _maxPulseWidth(maxPulseWidth),
    _attached(false),
    _currentDegrees(0.0f) {}

void ServoMotor::setup() {
    if (!_attached) {
        _servo.attach(_pin, _minPulseWidth, _maxPulseWidth);
        _attached = true;
        _servo.write(static_cast<int>(_currentDegrees));
    }
}

void ServoMotor::writeAngle(float degrees) {
    if (!_attached) {
        setup();
    }

    _currentDegrees = constrain(degrees, 0.0f, 180.0f);
    _servo.write(static_cast<int>(_currentDegrees));
}

void ServoMotor::detach() {
    if (_attached) {
        _servo.detach();
        _attached = false;
    }
}
