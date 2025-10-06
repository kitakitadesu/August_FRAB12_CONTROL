#include "servo_command_subscriber.hpp"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){/*error*/}}

ServoCommandSubscriber* ServoCommandSubscriber::_instance = nullptr;

ServoCommandSubscriber::ServoCommandSubscriber(ServoMotor& servo)
    : _servo(servo), _speedDegreesPerSecond(90.0f) {  // Default speed: 90 degrees/second
    _instance = this;
}

ServoCommandSubscriber::~ServoCommandSubscriber() {
    // Cleanup if needed
}

void ServoCommandSubscriber::setup(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor) {
    _servo.setup();

    RCCHECK(rclc_subscription_init_default(
        &_subscriber,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "servo_position"));

    RCCHECK(rclc_executor_add_subscription(executor, &_subscriber, &_msg, subscription_callback, ON_NEW_DATA));
}

void ServoCommandSubscriber::subscription_callback(const void* msgin) {
    const std_msgs__msg__Int32* msg = static_cast<const std_msgs__msg__Int32*>(msgin);
    if (_instance != nullptr) {
        _instance->handleServoCommand(msg);
    }
}

void ServoCommandSubscriber::handleServoCommand(const std_msgs__msg__Int32* msg) {
    int32_t value = msg->data;

    if (value >= 0) {
        // Positive value: set position with current speed
        const float target_degrees = static_cast<float>(value);
        _servo.writeAngleWithSpeed(target_degrees, _speedDegreesPerSecond);
    } else {
        // Negative value: set speed (absolute value)
        _speedDegreesPerSecond = static_cast<float>(-value);
        Serial.print("Servo speed set to: ");
        Serial.print(_speedDegreesPerSecond);
        Serial.println(" degrees/second");
    }
}
