#include "toggle_led_subscriber.hpp"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){/*error*/}}

ToggleLedSubscriber* ToggleLedSubscriber::_instance = nullptr;

ToggleLedSubscriber::ToggleLedSubscriber() : _led_enabled(false) {
    _instance = this;
}

ToggleLedSubscriber::~ToggleLedSubscriber() {
    // Cleanup if needed
}

void ToggleLedSubscriber::setup(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor) {
    // Create subscriber
    RCCHECK(rclc_subscription_init_default(
        &_subscriber,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "toggle_led"));

    // Add subscription to executor
    RCCHECK(rclc_executor_add_subscription(executor, &_subscriber, &_msg, subscription_callback, ON_NEW_DATA));
}

void ToggleLedSubscriber::subscription_callback(const void * msgin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    if (_instance) {
        _instance->handle_toggle_led(msg);
    }
}

void ToggleLedSubscriber::handle_toggle_led(const std_msgs__msg__Bool* msg) {
    _led_enabled = msg->data;
}
