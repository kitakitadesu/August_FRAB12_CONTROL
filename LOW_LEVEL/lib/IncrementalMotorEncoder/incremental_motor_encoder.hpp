#ifndef INCREMENTAL_MOTOR_ENCODER_H
#define INCREMENTAL_MOTOR_ENCODER_H

#include <Arduino.h>
#include "../DCMotor/dc_motor.hpp"
#include "../PIDController/pid_controller.hpp"

class IncrementalMotorEncoder {
public:
    IncrementalMotorEncoder(DCMotor& motor, int encoderPinA, int encoderPinB);

    void setup();
    void setSpeed(uint8_t speed);
    void setTargetSpeed(float target_speed);
    void updateSpeedControl();
    void forward();
    void backward();
    void stop();
    void brake();

    long getPosition();
    void resetPosition();
    float getCurrentSpeed();

private:
    DCMotor& _motor;
    int _encoderPinA;
    int _encoderPinB;
    volatile long _position;
    volatile int _lastStateA;
    volatile int _lastStateB;

    PIDController _pid;
    float _target_speed;
    unsigned long _last_time;
    long _last_position;
    float _current_speed;
    bool _direction; // true for forward, false for backward

    static IncrementalMotorEncoder* _instances[4];  // For interrupt handling
    static int _instance_count;

    static void handleEncoderInterruptA1();
    static void handleEncoderInterruptA2();
    static void handleEncoderInterruptA3();
    static void handleEncoderInterruptA4();
    static void handleEncoderInterruptB1();
    static void handleEncoderInterruptB2();
    static void handleEncoderInterruptB3();
    static void handleEncoderInterruptB4();
    void updatePosition();
};

#endif