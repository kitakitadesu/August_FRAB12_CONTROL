
#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "../lib/DCMotor/dc_motor.hpp"
#include "../lib/IncrementalMotorEncoder/incremental_motor_encoder.hpp"

#include "nodes/publishers/encoder_publisher.hpp"
#include "nodes/subscribers/cmd_vel_subscriber.hpp"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error Only avaliable for Arduino framework with serial transport.
#endif

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Motor and Encoder instances
DCMotor motorM1(0, 1, 2);    // GP0, GP1, GP2
DCMotor motorM2(3, 4, 5);    // GP3, GP4, GP5
DCMotor motorM3(6, 7, 8);    // GP6, GP7, GP8
DCMotor motorM4(9, 10, 11);  // GP9, GP10, GP11

IncrementalMotorEncoder encoderM1(motorM1, 12, 13);   // GP12, GP13
IncrementalMotorEncoder encoderM2(motorM2, 14, 15);   // GP14, GP15
IncrementalMotorEncoder encoderM3(motorM3, 16, 17);   // GP16, GP17
IncrementalMotorEncoder encoderM4(motorM4, 18, 19);   // GP18, GP19

// ROS2 Node instances
EncoderPublisher encoder_publisher(encoderM1, encoderM2);
CmdVelSubscriber cmd_vel_subscriber(encoderM1, encoderM2, encoderM3, encoderM4);

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  Serial.println("Bocchi The Rock!");
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  rclc_support_init(&support, 0, NULL, &allocator);

  // create node
  rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support);

  // create executor (now handles 2 handles: timer and subscriber)
  rclc_executor_init(&executor, &support.context, 2, &allocator);

  // Setup ROS2 nodes
  encoder_publisher.setup(&node, &support, &executor);
  cmd_vel_subscriber.setup(&node, &support, &executor);

  // Initialize motors and encoders
  motorM1.setup();
  motorM2.setup();
  motorM3.setup();
  motorM4.setup();

  encoderM1.setup();
  encoderM2.setup();
  encoderM3.setup();
  encoderM4.setup();
}

void loop() {
  delay(100);
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
