#include <Arduino.h>
#include "led_status_publisher.hpp"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){/*error*/}}

LedStatusPublisher* LedStatusPublisher::_instance = nullptr;

LedStatusPublisher::LedStatusPublisher() {
    _instance = this;
}

LedStatusPublisher::~LedStatusPublisher() {
    // Cleanup if needed
}

void LedStatusPublisher::setup(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor) {
    // Create publisher
    RCCHECK(rclc_publisher_init_default(
        &_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "led_status"));

    // Create timer
    const unsigned int timer_timeout = 500;  // Publish every 500ms
    RCCHECK(rclc_timer_init_default(
        &_timer,
        support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    // Add timer to executor
    RCCHECK(rclc_executor_add_timer(executor, &_timer));

    // Initialize message
    _msg.data = false;
}

void LedStatusPublisher::publish(bool led_state) {
    _msg.data = led_state;
    RCCHECK(rcl_publish(&_publisher, &_msg, NULL));
}

void LedStatusPublisher::timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL && _instance) {
        // Read current LED state from the pin
        bool current_led_state = digitalRead(LED_BUILTIN);
        _instance->publish(current_led_state);
    }
}