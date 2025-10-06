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

    // Speed control methods
    void setTargetSpeed(float targetSpeed);  // Target speed in encoder counts per second
    void updateSpeedControl();  // Call this periodically to update PID control
    float getCurrentSpeed();  // Get current speed in encoder counts per second

private:
    DCMotor& _motor;
    int _encoderPinA;
    int _encoderPinB;
    volatile long _position;
    volatile int _lastStateA;
    volatile int _lastStateB;

    // Speed control variables
    float _targetSpeed;
    float _currentSpeed;
    unsigned long _lastUpdateTime;
    long _lastPosition;
    float _integral;
    float _previousError;

    // PID constants (tune these)
    const float Kp = 0.5f;
    const float Ki = 0.1f;
    const float Kd = 0.01f;

    static IncrementalMotorEncoder* _instance;  // For interrupt handling

    static void handleEncoderInterrupt();
    void updatePosition();
    void updateCurrentSpeed();
};

#endif