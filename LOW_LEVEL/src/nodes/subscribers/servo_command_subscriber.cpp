#include "servo_command_subscriber.hpp"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

ServoCommandSubscriber* ServoCommandSubscriber::_instance = nullptr;

ServoCommandSubscriber::ServoCommandSubscriber(ServoMotor& servo)
    : _servo(servo) {  // Default 90 degrees per second
    _instance = this;
}

ServoCommandSubscriber::~ServoCommandSubscriber() {
    // Cleanup if needed
}

void ServoCommandSubscriber::setup(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor) {
    // Create servo_position subscriber
    RCCHECK(rclc_subscription_init_default(
        &_subscriber,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "servo_position"));

    // Add servo_position subscription to executor
    RCCHECK(rclc_executor_add_subscription(executor, &_subscriber, &_msg, subscription_callback, ON_NEW_DATA));
}

void ServoCommandSubscriber::subscription_callback(const void * msgin) {
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    if (_instance) {
        _instance->handleServoCommand(msg);
    }
}

void ServoCommandSubscriber::handleServoCommand(const std_msgs__msg__Int32* msg) {
    float angle = static_cast<float>(msg->data);
    _servo.writeAngle(angle);
}