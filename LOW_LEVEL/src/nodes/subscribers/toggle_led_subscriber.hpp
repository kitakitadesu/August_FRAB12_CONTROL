#ifndef TOGGLE_LED_SUBSCRIBER_H
#define TOGGLE_LED_SUBSCRIBER_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>

class ToggleLedSubscriber {
public:
    ToggleLedSubscriber();
    ~ToggleLedSubscriber();

    void setup(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor);
    bool isLedEnabled() const { return _led_enabled; }

private:
    rcl_subscription_t _subscriber;
    std_msgs__msg__Bool _msg;
    bool _led_enabled;

    static ToggleLedSubscriber* _instance;
    static void subscription_callback(const void * msgin);
    void handle_toggle_led(const std_msgs__msg__Bool* msg);
};

#endif
