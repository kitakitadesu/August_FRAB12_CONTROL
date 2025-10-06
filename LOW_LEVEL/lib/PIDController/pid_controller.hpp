#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float min_output = -255.0f, float max_output = 255.0f);
    ~PIDController();

    float update(float setpoint, float measured_value, float dt);
    void reset();

private:
    float _kp;
    float _ki;
    float _kd;
    float _min_output;
    float _max_output;
    float _integral;
    float _previous_error;
};

#endif