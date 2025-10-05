#include "dc_motor.hpp"

DCMotor::DCMotor(int pin1, int pin2) {
    _pin1 = pin1;
    _pin2 = pin2;
    _speed = 0;
}

DCMotor::~DCMotor() {
    stop();
    pinMode(_pin1, INPUT);
    pinMode(_pin2, INPUT);

}

void DCMotor::setup() {
    pinMode(_pin1, OUTPUT);
    pinMode(_pin2, OUTPUT);
    stop();
}

void DCMotor::setSpeed(uint8_t speed) {
    _speed = constrain(speed, 0, 255);
    // analogWrite(_enablePin, _speed);
}

void DCMotor::forward() {
    analogWrite(_pin1, _speed);
    digitalWrite(_pin2, LOW);
    Serial.println("Bocchi The Rock!");
}

void DCMotor::backward() {
    digitalWrite(_pin1, LOW);
    analogWrite(_pin2, _speed);
    Serial.println("Bocchi The Rock!");
}

void DCMotor::stop() {
    digitalWrite(_pin1, LOW);
    digitalWrite(_pin2, LOW);
}

void DCMotor::brake() {
    digitalWrite(_pin1, HIGH);
    digitalWrite(_pin2, HIGH);
}
