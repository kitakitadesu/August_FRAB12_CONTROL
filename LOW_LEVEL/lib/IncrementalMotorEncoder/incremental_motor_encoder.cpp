#include "incremental_motor_encoder.hpp"

IncrementalMotorEncoder* IncrementalMotorEncoder::_instance = nullptr;

IncrementalMotorEncoder::IncrementalMotorEncoder(DCMotor& motor, int encoderPinA, int encoderPinB)
    : _motor(motor), _encoderPinA(encoderPinA), _encoderPinB(encoderPinB), _position(0), _lastStateA(0), _lastStateB(0),
      _targetSpeed(0.0f), _currentSpeed(0.0f), _lastUpdateTime(0), _lastPosition(0), _integral(0.0f), _previousError(0.0f) {
    _instance = this;
}

void IncrementalMotorEncoder::setup() {
    pinMode(_encoderPinA, INPUT_PULLUP);
    pinMode(_encoderPinB, INPUT_PULLUP);

    // Read initial states
    _lastStateA = digitalRead(_encoderPinA);
    _lastStateB = digitalRead(_encoderPinB);

    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(_encoderPinA), handleEncoderInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(_encoderPinB), handleEncoderInterrupt, CHANGE);
}

void IncrementalMotorEncoder::setSpeed(uint8_t speed) {
    _motor.setSpeed(speed);
}

void IncrementalMotorEncoder::forward() {
    _motor.forward();
}

void IncrementalMotorEncoder::backward() {
    _motor.backward();
}

void IncrementalMotorEncoder::stop() {
    _motor.stop();
}

void IncrementalMotorEncoder::brake() {
    _motor.brake();
}

long IncrementalMotorEncoder::getPosition() {
    noInterrupts();
    long pos = _position;
    interrupts();
    return pos;
}

void IncrementalMotorEncoder::resetPosition() {
    noInterrupts();
    _position = 0;
    interrupts();
}

void IncrementalMotorEncoder::handleEncoderInterrupt() {
    if (_instance) {
        _instance->updatePosition();
    }
}

void IncrementalMotorEncoder::updatePosition() {
    int stateA = digitalRead(_encoderPinA);
    int stateB = digitalRead(_encoderPinB);

    // Quadrature decoding
    if (stateA != _lastStateA || stateB != _lastStateB) {
        if (stateA == HIGH) {
            if (stateB == LOW) {
                _position++;  // Forward
            } else {
                _position--;  // Backward
            }
        } else {
            if (stateB == HIGH) {
                _position++;  // Forward
            } else {
                _position--;  // Backward
            }
        }
    }

    _lastStateA = stateA;
    _lastStateB = stateB;
}

// Speed control methods
void IncrementalMotorEncoder::setTargetSpeed(float targetSpeed) {
    _targetSpeed = targetSpeed;
    if (_targetSpeed == 0.0f) {
        stop();
    } else if (_targetSpeed > 0.0f) {
        forward();
    } else {
        backward();
    }
}

void IncrementalMotorEncoder::updateSpeedControl() {
    updateCurrentSpeed();
    float error = _targetSpeed - _currentSpeed;
    _integral += error;
    float derivative = error - _previousError;
    float output = Kp * error + Ki * _integral + Kd * derivative;
    _previousError = error;
    int pwm = constrain((int)output, -255, 255);
    setSpeed(abs(pwm));
}

float IncrementalMotorEncoder::getCurrentSpeed() {
    return _currentSpeed;
}

void IncrementalMotorEncoder::updateCurrentSpeed() {
    unsigned long currentTime = millis();
    long currentPosition = getPosition();
    unsigned long deltaTime = currentTime - _lastUpdateTime;
    if (deltaTime > 0) {
        long deltaPosition = currentPosition - _lastPosition;
        _currentSpeed = (float)deltaPosition / (deltaTime / 1000.0f);
    }
    _lastUpdateTime = currentTime;
    _lastPosition = currentPosition;
}