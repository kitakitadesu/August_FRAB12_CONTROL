#ifndef DCMOTOR_H
#define DCMOTOR_H

#include <Arduino.h>

class DCMotor {
public:
    DCMotor(int pin1, int pin2);
    ~DCMotor();

    void setup();
    void setSpeed(uint8_t speed);
    void forward();
    void backward();
    void stop();
    void brake();

private:
    int _pin1;
    int _pin2;
    int _speed;
};

#endif
