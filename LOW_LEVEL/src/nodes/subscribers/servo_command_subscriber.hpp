#ifndef SERVO_COMMAND_SUBSCRIBER_HPP
#define SERVO_COMMAND_SUBSCRIBER_HPP

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

#include "../../lib/ServoMotor/servo_motor.hpp"

class ServoCommandSubscriber {
public:
    explicit ServoCommandSubscriber(ServoMotor& servo);
    ~ServoCommandSubscriber();

    void setup(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor);

private:
    ServoMotor& _servo;
    rcl_subscription_t _subscriber;
    std_msgs__msg__Int32 _msg;
    rcl_subscription_t _speed_subscriber;
    std_msgs__msg__Float32 _speed_msg;
    float _speedDegreesPerSecond;

    static ServoCommandSubscriber* _instance;
    static void subscription_callback(const void* msgin);
    static void speed_subscription_callback(const void* msgin);
    void handleServoCommand(const std_msgs__msg__Int32* msg);
    void handleServoSpeed(const std_msgs__msg__Float32* msg);
};

#endif