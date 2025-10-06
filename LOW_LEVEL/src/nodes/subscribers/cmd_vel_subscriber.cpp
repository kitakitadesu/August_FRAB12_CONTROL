#include "cmd_vel_subscriber.hpp"
#include <math.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){/*error*/}}

namespace {
// Tune this value to match the robot's footprint (L + W) / wheel_radius.
constexpr float kRotationGain = 0.5f;
// Motor deadzone to prevent jitter while stopping.
constexpr float kDeadzone = 0.02f;
// Set to true when a positive command should spin the wheel "backward" due to wiring or mirrored mounting.
constexpr bool kMotorPolarity[4] = {
    false,  // Motor M1 (top-left)
    false,  // Motor M2 (bottom-left)
    false,  // Motor M3 (top-right)
    false   // Motor M4 (bottom-right)
};
}

CmdVelSubscriber* CmdVelSubscriber::_instance = nullptr;

CmdVelSubscriber::CmdVelSubscriber(IncrementalMotorEncoder& encoder_m1, IncrementalMotorEncoder& encoder_m2,
                                   IncrementalMotorEncoder& encoder_m3, IncrementalMotorEncoder& encoder_m4)
    : _encoder_m1(encoder_m1), _encoder_m2(encoder_m2), _encoder_m3(encoder_m3), _encoder_m4(encoder_m4),
      _speed_scaling(1000.0f) {  // Default scaling: 1000 encoder counts/second at full speed
    _instance = this;
}

CmdVelSubscriber::~CmdVelSubscriber() {
    // Cleanup if needed
}

void CmdVelSubscriber::setup(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor) {
    // Create cmd_vel subscriber
    RCCHECK(rclc_subscription_init_default(
        &_subscriber,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    // Add cmd_vel subscription to executor
    RCCHECK(rclc_executor_add_subscription(executor, &_subscriber, &_msg, subscription_callback, ON_NEW_DATA));

    // Create motor speed subscriber
    RCCHECK(rclc_subscription_init_default(
        &_speed_subscriber,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "motor_speed"));

    // Add speed subscription to executor
    RCCHECK(rclc_executor_add_subscription(executor, &_speed_subscriber, &_speed_msg, speed_subscription_callback, ON_NEW_DATA));
}

void CmdVelSubscriber::setSpeedScaling(float scaling) {
    _speed_scaling = constrain(scaling, 100.0f, 5000.0f); // Reasonable limits
    Serial.print("Motor speed scaling set to: ");
    Serial.println(_speed_scaling);
}

void CmdVelSubscriber::subscription_callback(const void * msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    if (_instance) {
        _instance->handle_cmd_vel(msg);
    }
}

void CmdVelSubscriber::speed_subscription_callback(const void * msgin) {
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    if (_instance) {
        _instance->handle_speed_scaling(msg);
    }
}

void CmdVelSubscriber::handle_speed_scaling(const std_msgs__msg__Float32* msg) {
    setSpeedScaling(msg->data);
}

void CmdVelSubscriber::handle_cmd_vel(const geometry_msgs__msg__Twist* msg) {
    const float vx = msg->linear.x;   // Forward (+X)
    const float vy = msg->linear.y;   // Left (+Y)
    const float wz = msg->angular.z;  // Counter-clockwise (+Z)

    // Inverse kinematics for mecanum drive (normalized, wheel order: FL, FR, RL, RR)
    const float rotation_component = wz * kRotationGain;

    const float front_left  = vx - vy - rotation_component; // M1 (top-left)
    const float rear_left   = vx + vy - rotation_component; // M2 (bottom-left)
    const float front_right = vx + vy + rotation_component; // M3 (top-right)
    const float rear_right  = vx - vy + rotation_component; // M4 (bottom-right)

    float wheel_commands[4] = {
        front_left,
        rear_left,
        front_right,
        rear_right
    };

    // Normalize so the largest magnitude is 1.0 to keep relative ratios
    float max_command = fabsf(wheel_commands[0]);
    for (int i = 1; i < 4; ++i) {
        float magnitude = fabsf(wheel_commands[i]);
        if (magnitude > max_command) {
            max_command = magnitude;
        }
    }

    if (max_command < 1.0f) {
        max_command = 1.0f;
    }

    wheel_commands[0] /= max_command;
    wheel_commands[1] /= max_command;
    wheel_commands[2] /= max_command;
    wheel_commands[3] /= max_command;

    commandWheel(_encoder_m1, wheel_commands[0], kMotorPolarity[0]); // M1 top-left
    commandWheel(_encoder_m2, wheel_commands[1], kMotorPolarity[1]); // M2 bottom-left
    commandWheel(_encoder_m3, wheel_commands[2], kMotorPolarity[2]); // M3 top-right
    commandWheel(_encoder_m4, wheel_commands[3], kMotorPolarity[3]); // M4 bottom-right
}

void CmdVelSubscriber::commandWheel(IncrementalMotorEncoder& encoder, float command, bool inverted_polarity) {
    const float clipped = constrain(command, -1.0f, 1.0f);
    const float magnitude = fabsf(clipped);

    if (magnitude < kDeadzone) {
        encoder.setTargetSpeed(0.0f);
        return;
    }

    // Set target speed proportional to command using adjustable scaling
    float target_speed = clipped * _speed_scaling;
    if (inverted_polarity) {
        target_speed = -target_speed;
    }

    encoder.setTargetSpeed(target_speed);
}