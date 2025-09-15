#include "dc_motor.hpp"

DCMotor::DCMotor(int pin1, int pin2, int enablePin) {
    _pin1 = pin1;
    _pin2 = pin2;
    _enablePin = enablePin;
    _speed = 0;
}

DCMotor::~DCMotor() {
    stop();
    pinMode(_pin1, INPUT);
    pinMode(_pin2, INPUT);
    pinMode(_enablePin, INPUT);
}

void DCMotor::setup() {
    pinMode(_pin1, OUTPUT);
    pinMode(_pin2, OUTPUT);
    pinMode(_enablePin, OUTPUT);
    stop();
}

void DCMotor::setSpeed(uint8_t speed) {
    _speed = constrain(speed, 0, 255);
    analogWrite(_enablePin, _speed);
}

void DCMotor::forward() {
    digitalWrite(_pin1, HIGH);
    digitalWrite(_pin2, LOW);
}

void DCMotor::backward() {
    digitalWrite(_pin1, LOW);
    digitalWrite(_pin2, HIGH);
}

void DCMotor::stop() {
    digitalWrite(_pin1, LOW);
    digitalWrite(_pin2, LOW);
    analogWrite(_enablePin, 0);
}

void DCMotor::brake() {
    digitalWrite(_pin1, HIGH);
    digitalWrite(_pin2, HIGH);
    analogWrite(_enablePin, 0);
}
