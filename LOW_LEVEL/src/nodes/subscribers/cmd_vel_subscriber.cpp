#include "cmd_vel_subscriber.hpp"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){/*error*/}}

CmdVelSubscriber* CmdVelSubscriber::_instance = nullptr;

CmdVelSubscriber::CmdVelSubscriber(IncrementalMotorEncoder& encoder_m1, IncrementalMotorEncoder& encoder_m2,
                                   IncrementalMotorEncoder& encoder_m3, IncrementalMotorEncoder& encoder_m4)
    : _encoder_m1(encoder_m1), _encoder_m2(encoder_m2), _encoder_m3(encoder_m3), _encoder_m4(encoder_m4) {
    _instance = this;
}

CmdVelSubscriber::~CmdVelSubscriber() {
    // Cleanup if needed
}

void CmdVelSubscriber::setup(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor) {
    // Create subscriber
    RCCHECK(rclc_subscription_init_default(
        &_subscriber,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    // Add subscription to executor
    RCCHECK(rclc_executor_add_subscription(executor, &_subscriber, &_msg, subscription_callback, ON_NEW_DATA));
}

void CmdVelSubscriber::subscription_callback(const void * msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    if (_instance) {
        _instance->handle_cmd_vel(msg);
    }
}

void CmdVelSubscriber::handle_cmd_vel(const geometry_msgs__msg__Twist* msg) {
    // Extract velocities
    float linear_x = msg->linear.x;
    float angular_z = msg->angular.z;

    // Differential drive calculation
    // Assuming M1,M3 = left side, M2,M4 = right side
    float left_speed = linear_x - angular_z;
    float right_speed = linear_x + angular_z;

    // Scale to motor speed range (0-255)
    // Assuming input range is -1.0 to 1.0
    int left_motor_speed = constrain(abs(left_speed) * 255, 0, 255);
    int right_motor_speed = constrain(abs(right_speed) * 255, 0, 255);

    // Control left side motors (M1, M3)
    if (left_speed > 0.01) {
        _encoder_m1.setSpeed(left_motor_speed);
        _encoder_m1.forward();
        _encoder_m3.setSpeed(left_motor_speed);
        _encoder_m3.forward();
    } else if (left_speed < -0.01) {
        _encoder_m1.setSpeed(left_motor_speed);
        _encoder_m1.backward();
        _encoder_m3.setSpeed(left_motor_speed);
        _encoder_m3.backward();
    } else {
        _encoder_m1.stop();
        _encoder_m3.stop();
    }

    // Control right side motors (M2, M4)
    if (right_speed > 0.01) {
        _encoder_m2.setSpeed(right_motor_speed);
        _encoder_m2.forward();
        _encoder_m4.setSpeed(right_motor_speed);
        _encoder_m4.forward();
    } else if (right_speed < -0.01) {
        _encoder_m2.setSpeed(right_motor_speed);
        _encoder_m2.backward();
        _encoder_m4.setSpeed(right_motor_speed);
        _encoder_m4.backward();
    } else {
        _encoder_m2.stop();
        _encoder_m4.stop();
    }
}