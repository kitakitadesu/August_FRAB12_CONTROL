#ifndef ENCODER_PUBLISHER_H
#define ENCODER_PUBLISHER_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

#include "../../lib/IncrementalMotorEncoder/incremental_motor_encoder.hpp"

class EncoderPublisher {
public:
    EncoderPublisher(IncrementalMotorEncoder& encoder_m1, IncrementalMotorEncoder& encoder_m2);
    ~EncoderPublisher();

    void setup(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor);
    void publish();

private:
    IncrementalMotorEncoder& _encoder_m1;
    IncrementalMotorEncoder& _encoder_m2;

    rcl_publisher_t _publisher_m1;
    rcl_publisher_t _publisher_m2;
    rcl_timer_t _timer;

    std_msgs__msg__Int32 _msg_m1;
    std_msgs__msg__Int32 _msg_m2;

    static EncoderPublisher* _instance;
    static void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
};

#endif