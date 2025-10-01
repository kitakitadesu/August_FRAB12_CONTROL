#ifndef INCREMENTAL_MOTOR_ENCODER_H
#define INCREMENTAL_MOTOR_ENCODER_H

#include <Arduino.h>
#include "../DCMotor/dc_motor.hpp"

class IncrementalMotorEncoder {
public:
    IncrementalMotorEncoder(DCMotor& motor, int encoderPinA, int encoderPinB);

    void setup();
    void setSpeed(uint8_t speed);
    void forward();
    void backward();
    void stop();
    void brake();

    long getPosition();
    void resetPosition();

private:
    DCMotor& _motor;
    int _encoderPinA;
    int _encoderPinB;
    volatile long _position;
    volatile int _lastStateA;
    volatile int _lastStateB;

    static IncrementalMotorEncoder* _instance;  // For interrupt handling

    static void handleEncoderInterrupt();
    void updatePosition();
};

#endif