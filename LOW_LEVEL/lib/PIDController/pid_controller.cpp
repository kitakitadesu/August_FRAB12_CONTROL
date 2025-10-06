#include "pid_controller.hpp"

PIDController::PIDController(float kp, float ki, float kd, float min_output, float max_output)
    : _kp(kp), _ki(ki), _kd(kd), _min_output(min_output), _max_output(max_output),
      _integral(0.0f), _previous_error(0.0f) {
}

PIDController::~PIDController() {
    // Cleanup if needed
}

float PIDController::update(float setpoint, float measured_value, float dt) {
    float error = setpoint - measured_value;

    // Proportional term
    float p_term = _kp * error;

    // Integral term
    _integral += error * dt;
    float i_term = _ki * _integral;

    // Derivative term
    float derivative = (error - _previous_error) / dt;
    float d_term = _kd * derivative;

    // Calculate output
    float output = p_term + i_term + d_term;

    // Clamp output
    if (output > _max_output) {
        output = _max_output;
    } else if (output < _min_output) {
        output = _min_output;
    }

    // Update previous error
    _previous_error = error;

    return output;
}

void PIDController::reset() {
    _integral = 0.0f;
    _previous_error = 0.0f;
}