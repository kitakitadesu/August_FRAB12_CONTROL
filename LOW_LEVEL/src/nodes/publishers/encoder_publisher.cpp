#include "encoder_publisher.hpp"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){/*error*/}}

EncoderPublisher* EncoderPublisher::_instance = nullptr;

EncoderPublisher::EncoderPublisher(IncrementalMotorEncoder& encoder_m1, IncrementalMotorEncoder& encoder_m2)
    : _encoder_m1(encoder_m1), _encoder_m2(encoder_m2) {
    _instance = this;
}

EncoderPublisher::~EncoderPublisher() {
    // Cleanup if needed
}

void EncoderPublisher::setup(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor) {
    // Create publishers
    RCCHECK(rclc_publisher_init_default(
        &_publisher_m1,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "encoder_m1_position"));

    RCCHECK(rclc_publisher_init_default(
        &_publisher_m2,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "encoder_m2_position"));

    // Create timer
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default(
        &_timer,
        support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    // Add timer to executor
    RCCHECK(rclc_executor_add_timer(executor, &_timer));

    // Initialize messages
    _msg_m1.data = 0;
    _msg_m2.data = 0;
}

void EncoderPublisher::publish() {
    // This method can be called directly if needed, but timer handles regular publishing
    _msg_m1.data = _encoder_m1.getPosition();
    RCCHECK(rcl_publish(&_publisher_m1, &_msg_m1, NULL));

    _msg_m2.data = _encoder_m2.getPosition();
    RCCHECK(rcl_publish(&_publisher_m2, &_msg_m2, NULL));
}

void EncoderPublisher::timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL && _instance) {
        _instance->publish();
    }
}