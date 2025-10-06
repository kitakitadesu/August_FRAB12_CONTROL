
#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "../lib/DCMotor/dc_motor.hpp"
#include "../lib/IncrementalMotorEncoder/incremental_motor_encoder.hpp"

#include "nodes/publishers/encoder_publisher.hpp"
#include "nodes/publishers/led_status_publisher.hpp"
#include "nodes/subscribers/cmd_vel_subscriber.hpp"
#include "nodes/subscribers/toggle_led_subscriber.hpp"
#include "nodes/subscribers/servo_command_subscriber.hpp"
#include "../lib/ServoMotor/servo_motor.hpp"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error Only avaliable for Arduino framework with serial transport.
#endif

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Motor and Encoder instances
DCMotor motorM1(0, 1, 2);    // GP0, GP1, GP2
DCMotor motorM2(6, 7, 8);    // GP6, GP7, GP8 - top-right
DCMotor motorM3(3, 4, 5);    // GP3, GP4, GP5 - bottom-left
DCMotor motorM4(9, 10, 11);  // GP9, GP10, GP11

IncrementalMotorEncoder encoderM1(motorM1, 12, 13);   // GP12, GP13
IncrementalMotorEncoder encoderM2(motorM2, 16, 17);   // GP16, GP17 - top-right
IncrementalMotorEncoder encoderM3(motorM3, 14, 15);   // GP14, GP15 - bottom-left
IncrementalMotorEncoder encoderM4(motorM4, 18, 19);   // GP18, GP19

ServoMotor servo1(20);  // GP20

// ROS2 Node instances
EncoderPublisher encoder_publisher(encoderM1, encoderM2);
LedStatusPublisher led_status_publisher;
CmdVelSubscriber cmd_vel_subscriber(encoderM1, encoderM2, encoderM3, encoderM4);
ToggleLedSubscriber toggle_led_subscriber;
ServoCommandSubscriber servo_command_subscriber(servo1);

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    Serial.println("ERROR: micro-ROS initialization failed!");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
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
  Serial.println("Allocator created");

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  Serial.println("Support initialized");

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));
  Serial.println("Node created");

  // create executor (now handles 7 handles: two publishers and five subscribers)
  RCCHECK(rclc_executor_init(&executor, &support.context, 7, &allocator));
  Serial.println("Executor created");

  // Setup ROS2 nodes
  encoder_publisher.setup(&node, &support, &executor);
  Serial.println("Encoder publisher setup");
  led_status_publisher.setup(&node, &support, &executor);
  Serial.println("LED status publisher setup");
  cmd_vel_subscriber.setup(&node, &support, &executor);
  Serial.println("Cmd vel subscriber setup");
  toggle_led_subscriber.setup(&node, &support, &executor);
  Serial.println("Toggle LED subscriber setup");
  servo_command_subscriber.setup(&node, &support, &executor);
  Serial.println("Servo command subscriber setup");

  // Initialize motors and encoders
  motorM1.setup();
  motorM2.setup();
  motorM3.setup();
  motorM4.setup();

  encoderM1.setup();
  encoderM2.setup();
  encoderM3.setup();
  encoderM4.setup();
  servo1.setup();
  servo1.writeAngle(0.0f);
  motorM1.setSpeed(150);
  // motorM1.forward();
  //   motorM2.setSpeed(150);
  // motorM2.forward();
  //   motorM3.setSpeed(150);
  // motorM3.forward();
  //   motorM4.setSpeed(150);
  // motorM4.forward();

  // Setup built-in LED for blinking
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("Setup complete!");
}

void loop() {
  delay(100);
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

  // Update speed control for all motors
  encoderM1.updateSpeedControl();
  encoderM2.updateSpeedControl();
  encoderM3.updateSpeedControl();
  encoderM4.updateSpeedControl();

  // Update servo position
  servo1.update();

  digitalWrite(LED_BUILTIN, toggle_led_subscriber.isLedEnabled());
}
