#ifndef SERVO_MOTOR_HPP
#define SERVO_MOTOR_HPP

#include <Arduino.h>
#include <Servo.h>

class ServoMotor {
public:
    explicit ServoMotor(uint8_t controlPin, uint16_t minPulseWidth = 500, uint16_t maxPulseWidth = 2500);

    void setup();
    void writeAngle(float degrees);
    void writeAngleWithSpeed(float degrees, float degreesPerSecond);
    void update();
    void detach();
    float currentAngle() const { return _currentDegrees; }

private:
    Servo _servo;
    uint8_t _pin;
    uint16_t _minPulseWidth;
    uint16_t _maxPulseWidth;
    bool _attached;
    float _currentDegrees;
    float _targetDegrees;
    float _speedDegreesPerSecond;
    unsigned long _lastUpdateTime;
};

#endif