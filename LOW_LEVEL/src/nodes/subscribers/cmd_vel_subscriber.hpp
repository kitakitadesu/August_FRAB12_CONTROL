#ifndef CMD_VEL_SUBSCRIBER_H
#define CMD_VEL_SUBSCRIBER_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

#include "../../lib/IncrementalMotorEncoder/incremental_motor_encoder.hpp"

class CmdVelSubscriber {
public:
    CmdVelSubscriber(IncrementalMotorEncoder& encoder_m1, IncrementalMotorEncoder& encoder_m2,
                     IncrementalMotorEncoder& encoder_m3, IncrementalMotorEncoder& encoder_m4);
    ~CmdVelSubscriber();

    void setup(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor);

private:
    IncrementalMotorEncoder& _encoder_m1;
    IncrementalMotorEncoder& _encoder_m2;
    IncrementalMotorEncoder& _encoder_m3;
    IncrementalMotorEncoder& _encoder_m4;

    rcl_subscription_t _subscriber;
    geometry_msgs__msg__Twist _msg;

    static CmdVelSubscriber* _instance;
    static void subscription_callback(const void * msgin);
    void handle_cmd_vel(const geometry_msgs__msg__Twist* msg);
    void commandWheel(IncrementalMotorEncoder& encoder, float command, bool inverted_polarity);
};

#endif