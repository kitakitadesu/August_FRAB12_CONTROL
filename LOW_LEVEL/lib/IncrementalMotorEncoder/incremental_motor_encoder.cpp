#include "incremental_motor_encoder.hpp"

IncrementalMotorEncoder* IncrementalMotorEncoder::_instances[4] = {nullptr, nullptr, nullptr, nullptr};
int IncrementalMotorEncoder::_instance_count = 0;

IncrementalMotorEncoder::IncrementalMotorEncoder(DCMotor& motor, int encoderPinA, int encoderPinB)
    : _motor(motor), _encoderPinA(encoderPinA), _encoderPinB(encoderPinB), _position(0), _lastStateA(0), _lastStateB(0),
      _pid(1.0f, 0.1f, 0.01f), _target_speed(0.0f), _last_time(0), _last_position(0), _current_speed(0.0f), _direction(true) {
    if (_instance_count < 4) {
        _instances[_instance_count] = this;
        _instance_count++;
    }
}

void IncrementalMotorEncoder::setup() {
    pinMode(_encoderPinA, INPUT_PULLUP);
    pinMode(_encoderPinB, INPUT_PULLUP);

    // Read initial states
    _lastStateA = digitalRead(_encoderPinA);
    _lastStateB = digitalRead(_encoderPinB);

    // Attach interrupts based on instance index
    int index = -1;
    for (int i = 0; i < 4; i++) {
        if (_instances[i] == this) {
            index = i;
            break;
        }
    }

    if (index >= 0) {
        switch (index) {
            case 0:
                attachInterrupt(digitalPinToInterrupt(_encoderPinA), handleEncoderInterruptA1, CHANGE);
                attachInterrupt(digitalPinToInterrupt(_encoderPinB), handleEncoderInterruptB1, CHANGE);
                break;
            case 1:
                attachInterrupt(digitalPinToInterrupt(_encoderPinA), handleEncoderInterruptA2, CHANGE);
                attachInterrupt(digitalPinToInterrupt(_encoderPinB), handleEncoderInterruptB2, CHANGE);
                break;
            case 2:
                attachInterrupt(digitalPinToInterrupt(_encoderPinA), handleEncoderInterruptA3, CHANGE);
                attachInterrupt(digitalPinToInterrupt(_encoderPinB), handleEncoderInterruptB3, CHANGE);
                break;
            case 3:
                attachInterrupt(digitalPinToInterrupt(_encoderPinA), handleEncoderInterruptA4, CHANGE);
                attachInterrupt(digitalPinToInterrupt(_encoderPinB), handleEncoderInterruptB4, CHANGE);
                break;
        }
    }
}

void IncrementalMotorEncoder::setSpeed(uint8_t speed) {
    _motor.setSpeed(speed);
}

void IncrementalMotorEncoder::setTargetSpeed(float target_speed) {
    _target_speed = target_speed;
    _pid.reset();
}

void IncrementalMotorEncoder::updateSpeedControl() {
    unsigned long current_time = millis();
    long current_position = getPosition();
    float dt = (current_time - _last_time) / 1000.0f; // Convert to seconds

    if (dt > 0.01f) { // Update at least every 10ms
        long position_diff = current_position - _last_position;
        _current_speed = position_diff / dt; // Speed in encoder counts per second

        Serial.print("Target Speed: ");
        Serial.print(_target_speed);
        Serial.print(", Current Speed: ");
        Serial.println(_current_speed);

        float pid_output = _pid.update(_target_speed, _current_speed, dt);
        uint8_t pwm = constrain(abs(pid_output), 0, 255);

        Serial.print("PID Output: ");
        Serial.print(pid_output);
        Serial.print(", PWM: ");
        Serial.println(pwm);

        _motor.setSpeed(pwm);

        if (_target_speed > 0) {
            _direction = true;
            _motor.forward();
        } else if (_target_speed < 0) {
            _direction = false;
            _motor.backward();
        } else {
            _motor.stop();
        }

        _last_time = current_time;
        _last_position = current_position;
    }
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

float IncrementalMotorEncoder::getCurrentSpeed() {
    return _current_speed;
}

void IncrementalMotorEncoder::handleEncoderInterruptA1() {
    if (_instances[0]) {
        _instances[0]->updatePosition();
    }
}

void IncrementalMotorEncoder::handleEncoderInterruptA2() {
    if (_instances[1]) {
        _instances[1]->updatePosition();
    }
}

void IncrementalMotorEncoder::handleEncoderInterruptA3() {
    if (_instances[2]) {
        _instances[2]->updatePosition();
    }
}

void IncrementalMotorEncoder::handleEncoderInterruptA4() {
    if (_instances[3]) {
        _instances[3]->updatePosition();
    }
}

void IncrementalMotorEncoder::handleEncoderInterruptB1() {
    if (_instances[0]) {
        _instances[0]->updatePosition();
    }
}

void IncrementalMotorEncoder::handleEncoderInterruptB2() {
    if (_instances[1]) {
        _instances[1]->updatePosition();
    }
}

void IncrementalMotorEncoder::handleEncoderInterruptB3() {
    if (_instances[2]) {
        _instances[2]->updatePosition();
    }
}

void IncrementalMotorEncoder::handleEncoderInterruptB4() {
    if (_instances[3]) {
        _instances[3]->updatePosition();
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