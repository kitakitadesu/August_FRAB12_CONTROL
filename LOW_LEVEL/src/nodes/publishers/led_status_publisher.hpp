#ifndef LED_STATUS_PUBLISHER_H
#define LED_STATUS_PUBLISHER_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>

class LedStatusPublisher {
public:
    LedStatusPublisher();
    ~LedStatusPublisher();

    void setup(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor);
    void publish(bool led_state);

private:
    rcl_publisher_t _publisher;
    rcl_timer_t _timer;

    std_msgs__msg__Bool _msg;

    static LedStatusPublisher* _instance;
    static void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
};

#endif