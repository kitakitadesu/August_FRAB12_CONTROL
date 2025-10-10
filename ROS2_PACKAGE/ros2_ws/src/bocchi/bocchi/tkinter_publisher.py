#!/usr/bin/env python3
"""
Tkinter-based ROS2 Publisher for Bocchi Robot Control
Provides a GUI interface for robot control using Tkinter instead of web UI.

Usage:
    python -m bocchi.tkinter_publisher
    or
    ros2 run bocchi tkinter_publisher

Controls:
- WASD keys: Movement control (W=forward, S=backward, A=left turn, D=right turn)
- F key: Toggle servo position (0° ↔ 180°)
- L key: Toggle LED on/off
- GUI buttons: Manual control for movement and servo/LED toggle
- Sliders: Adjust motor speed (0-2000) and servo speed (0-180)

The GUI provides real-time status updates and keyboard event logging.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool, Float32
import tkinter as tk
from tkinter import ttk
import threading
import time
import signal
import sys
import argparse

minimal_publisher = None

def signal_handler(sig, frame):
    """Handle graceful shutdown on SIGINT"""
    print("\nSIGINT (Ctrl+C) received! Performing graceful shutdown.")
    if minimal_publisher:
        minimal_publisher.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

class KeyStateManager:
    """Manages key state to prevent duplicate publications"""

    def __init__(self, debounce_time=0.05):
        self.debounce_time = debounce_time
        self.current_keys = set()
        self.servo_position = 0  # Track servo position (0 or 180)
        self.last_key_time = 0
        self.key_changed = False
        self.servo_changed = False

    def update_key(self, key_code, is_pressed=True):
        """Update key state and return if state changed"""
        current_time = time.time()

        # Handle F key for servo toggle
        if key_code == 102 and is_pressed:  # 'f' key
            self.servo_position = 180 if self.servo_position == 0 else 0
            self.servo_changed = True
            return True

        if is_pressed:
            if key_code not in self.current_keys:
                self.current_keys.add(key_code)
                self.key_changed = True
                self.last_key_time = current_time
                return True
        else:
            if key_code in self.current_keys:
                self.current_keys.remove(key_code)
                self.key_changed = True
                self.last_key_time = current_time
                return True

        return False

    def get_twist_if_changed(self):
        """Get Twist message if key state changed"""
        if self.key_changed:
            self.key_changed = False
            return self._calculate_twist()
        return None

    def _calculate_twist(self):
        """Calculate Twist message based on current pressed keys"""
        twist = Twist()

        # Key mappings based on WASD keys
        # W = forward (119), S = backward (115), A = turn left (97), D = turn right (100)

        # Linear velocity (forward/backward)
        if 119 in self.current_keys:  # 'w'
            twist.linear.x = 0.5
        elif 115 in self.current_keys:  # 's'
            twist.linear.x = -0.5
        else:
            twist.linear.x = 0.0

        # Angular velocity (left/right turn)
        if 97 in self.current_keys:  # 'a'
            twist.angular.z = 0.5
        elif 100 in self.current_keys:  # 'd'
            twist.angular.z = -0.5
        else:
            twist.angular.z = 0.0

        # All other fields remain 0.0 by default
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0

        return twist

    def get_servo_if_changed(self):
        """Get servo position if it changed"""
        if self.servo_changed:
            self.servo_changed = False
            return self.servo_position
        return None

class TkinterGUI:
    """Tkinter-based GUI for robot control"""

    def __init__(self, publisher_node):
        self.publisher_node = publisher_node
        self.root = tk.Tk()
        self.root.title("Bocchi Robot Controller - Tkinter GUI")
        self.root.geometry("600x500")
        self.root.resizable(True, True)

        # Key state tracking
        self.pressed_keys = set()

        # Status variables (initialize before setup_gui)
        self.led_state = tk.BooleanVar(value=False)
        self.servo_position = tk.IntVar(value=0)
        self.motor_speed = tk.DoubleVar(value=1000.0)
        self.servo_speed = tk.DoubleVar(value=90.0)

        # Setup GUI components
        self.setup_gui()

        # Bind keyboard events
        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.bind('<KeyRelease>', self.on_key_release)

        # Focus on root to capture keyboard events
        self.root.focus_set()

        # Update status periodically
        self.update_status()

    def setup_gui(self):
        """Setup the GUI components"""
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)

        # Title
        title_label = ttk.Label(main_frame, text="Bocchi Robot Controller",
                               font=("Arial", 16, "bold"))
        title_label.grid(row=0, column=0, columnspan=3, pady=(0, 20))

        # Movement control section
        movement_frame = ttk.LabelFrame(main_frame, text="Movement Control", padding="10")
        movement_frame.grid(row=1, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))

        # WASD control instructions
        ttk.Label(movement_frame, text="Use WASD keys for movement:").grid(row=0, column=0, sticky=tk.W)
        ttk.Label(movement_frame, text="W: Forward").grid(row=1, column=0, sticky=tk.W)
        ttk.Label(movement_frame, text="S: Backward").grid(row=2, column=0, sticky=tk.W)
        ttk.Label(movement_frame, text="A: Turn Left").grid(row=3, column=0, sticky=tk.W)
        ttk.Label(movement_frame, text="D: Turn Right").grid(row=4, column=0, sticky=tk.W)

        # Manual control buttons
        button_frame = ttk.Frame(movement_frame)
        button_frame.grid(row=1, column=1, rowspan=4, padx=(20, 0))

        ttk.Button(button_frame, text="Forward (W)",
                  command=lambda: self.manual_move('w')).grid(row=0, column=0, pady=2)
        ttk.Button(button_frame, text="Backward (S)",
                  command=lambda: self.manual_move('s')).grid(row=1, column=0, pady=2)
        ttk.Button(button_frame, text="Left (A)",
                  command=lambda: self.manual_move('a')).grid(row=2, column=0, pady=2)
        ttk.Button(button_frame, text="Right (D)",
                  command=lambda: self.manual_move('d')).grid(row=3, column=0, pady=2)
        ttk.Button(button_frame, text="Stop",
                  command=self.stop_movement).grid(row=4, column=0, pady=2)

        # Servo control section
        servo_frame = ttk.LabelFrame(main_frame, text="Servo Control", padding="10")
        servo_frame.grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))

        ttk.Label(servo_frame, text="Press 'F' key or use button to toggle servo:").grid(row=0, column=0, sticky=tk.W)
        ttk.Button(servo_frame, text="Toggle Servo (F)",
                  command=self.toggle_servo).grid(row=1, column=0, pady=5)

        # Speed control section
        speed_frame = ttk.LabelFrame(main_frame, text="Speed Control", padding="10")
        speed_frame.grid(row=3, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))

        # Motor speed
        ttk.Label(speed_frame, text="Motor Speed:").grid(row=0, column=0, sticky=tk.W)
        motor_scale = ttk.Scale(speed_frame, from_=0, to=2000, variable=self.motor_speed,
                               orient=tk.HORIZONTAL, command=self.update_motor_speed)
        motor_scale.grid(row=0, column=1, sticky=(tk.W, tk.E), padx=(10, 0))
        motor_scale.set(1000.0)
        ttk.Label(speed_frame, textvariable=self.motor_speed).grid(row=0, column=2, padx=(10, 0))

        # Servo speed
        ttk.Label(speed_frame, text="Servo Speed:").grid(row=1, column=0, sticky=tk.W)
        servo_scale = ttk.Scale(speed_frame, from_=0, to=180, variable=self.servo_speed,
                               orient=tk.HORIZONTAL, command=self.update_servo_speed)
        servo_scale.grid(row=1, column=1, sticky=(tk.W, tk.E), padx=(10, 0))
        servo_scale.set(90.0)
        ttk.Label(speed_frame, textvariable=self.servo_speed).grid(row=1, column=2, padx=(10, 0))

        # LED control section
        led_frame = ttk.LabelFrame(main_frame, text="LED Control", padding="10")
        led_frame.grid(row=4, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))

        ttk.Label(led_frame, text="Press 'L' key or use button to toggle LED:").grid(row=0, column=0, sticky=tk.W)
        ttk.Button(led_frame, text="Toggle LED (L)",
                  command=self.toggle_led).grid(row=1, column=0, pady=5)

        # Status section
        status_frame = ttk.LabelFrame(main_frame, text="Status", padding="10")
        status_frame.grid(row=5, column=0, columnspan=3, sticky=(tk.W, tk.E))

        self.status_text = tk.Text(status_frame, height=6, width=50, state=tk.DISABLED)
        scrollbar = ttk.Scrollbar(status_frame, orient=tk.VERTICAL, command=self.status_text.yview)
        self.status_text.configure(yscrollcommand=scrollbar.set)

        self.status_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))

        status_frame.columnconfigure(0, weight=1)
        status_frame.rowconfigure(0, weight=1)

        # Initial status
        self.log_status("Tkinter GUI initialized")
        self.log_status("Use WASD keys for movement, F for servo toggle, L for LED toggle")
        self.log_status("Or use the control buttons for manual operation")

    def on_key_press(self, event):
        """Handle key press events"""
        key_code = event.keycode
        if key_code not in self.pressed_keys:
            self.pressed_keys.add(key_code)
            self.publisher_node.update_key_state(key_code, True)
            self.log_status(f"Key pressed: {event.char} (code: {key_code})")

    def on_key_release(self, event):
        """Handle key release events"""
        key_code = event.keycode
        if key_code in self.pressed_keys:
            self.pressed_keys.remove(key_code)
            self.publisher_node.update_key_state(key_code, False)
            self.log_status(f"Key released: {event.char} (code: {key_code})")

    def manual_move(self, direction):
        """Handle manual movement button presses"""
        key_map = {'w': 119, 's': 115, 'a': 97, 'd': 100}
        if direction in key_map:
            key_code = key_map[direction]
            self.publisher_node.update_key_state(key_code, True)
            # Auto-release after a short delay for button presses
            self.root.after(200, lambda: self.publisher_node.update_key_state(key_code, False))
            self.log_status(f"Manual move: {direction.upper()}")

    def stop_movement(self):
        """Stop all movement"""
        # Release all movement keys
        for key in [119, 115, 97, 100]:  # w, s, a, d
            if key in self.pressed_keys:
                self.pressed_keys.discard(key)
            self.publisher_node.update_key_state(key, False)
        self.log_status("Movement stopped")

    def toggle_servo(self):
        """Toggle servo position"""
        self.publisher_node.update_key_state(102, True)  # 'f' key
        self.root.after(100, lambda: self.publisher_node.update_key_state(102, False))
        self.log_status("Servo toggle requested")

    def toggle_led(self):
        """Toggle LED state"""
        self.publisher_node.update_key_state(108, True)  # 'l' key
        self.root.after(100, lambda: self.publisher_node.update_key_state(108, False))
        self.log_status("LED toggle requested")

    def update_motor_speed(self, value):
        """Update motor speed when slider changes"""
        speed = float(value)
        self.publisher_node.publish_motor_speed(speed)
        self.log_status(f"Motor speed updated: {speed}")

    def update_servo_speed(self, value):
        """Update servo speed when slider changes"""
        speed = float(value)
        self.publisher_node.publish_servo_speed(speed)
        self.log_status(f"Servo speed updated: {speed}")

    def log_status(self, message):
        """Log message to status text area"""
        timestamp = time.strftime("%H:%M:%S")
        self.status_text.configure(state=tk.NORMAL)
        self.status_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.status_text.see(tk.END)
        self.status_text.configure(state=tk.DISABLED)

    def update_status(self):
        """Update status display periodically"""
        # This could be enhanced to show more real-time status
        self.root.after(1000, self.update_status)  # Update every second

    def run(self):
        """Start the GUI main loop"""
        self.root.mainloop()

class TkinterPublisher:
    """Tkinter GUI with ROS2 Publisher functionality"""

    def __init__(self):
        # ROS2 components will be initialized in a separate thread
        self.ros_node = None
        self.ros_thread = None
        self.running = False

        # Initialize Tkinter GUI first (must be in main thread)
        self.gui = TkinterGUI(self)

        # Start ROS2 in a separate thread
        self.start_ros_node()

    def start_ros_node(self):
        """Start ROS2 node in a separate thread"""
        def ros_main():
            try:
                rclpy.init()

                # Create the ROS2 node
                self.ros_node = RosNode()
                self.running = True

                print("ROS2 node started in background thread")
                rclpy.spin(self.ros_node)

            except Exception as e:
                print(f"Error in ROS thread: {e}")
            finally:
                if self.ros_node:
                    self.ros_node.destroy_node()
                try:
                    rclpy.shutdown()
                except Exception as e:
                    print(f"Error during ROS shutdown: {e}")

        self.ros_thread = threading.Thread(target=ros_main, daemon=True)
        self.ros_thread.start()

        # Give ROS2 time to initialize
        time.sleep(1)

    def publish_twist(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        """Publish twist message through ROS2 node"""
        if self.ros_node:
            self.ros_node.publish_twist(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z)

    def publish_servo(self, position):
        """Publish servo position through ROS2 node"""
        if self.ros_node:
            self.ros_node.publish_servo(position)

    def publish_motor_speed(self, speed):
        """Publish motor speed through ROS2 node"""
        if self.ros_node:
            self.ros_node.publish_motor_speed(speed)

    def publish_servo_speed(self, speed):
        """Publish servo speed through ROS2 node"""
        if self.ros_node:
            self.ros_node.publish_servo_speed(speed)

    def publish_led_toggle(self, state):
        """Publish LED toggle through ROS2 node"""
        if self.ros_node:
            self.ros_node.publish_led_toggle(state)

    def update_key_state(self, key_code, is_pressed):
        """Update key state in ROS2 node"""
        if self.ros_node:
            self.ros_node.update_key_state(key_code, is_pressed)

    def run_gui(self):
        """Run the Tkinter GUI main loop"""
        self.gui.run()

    def shutdown(self):
        """Shutdown the application"""
        self.running = False
        if self.ros_node:
            self.ros_node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"Error during shutdown: {e}")

class RosNode(Node):
    """ROS2 Node that handles all ROS2 operations"""

    def __init__(self):
        super().__init__('tkinter_publisher')

        # ROS2 Publishers
        self.twist_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.servo_publisher = self.create_publisher(Int32, 'servo_position', 10)
        self.motor_speed_publisher = self.create_publisher(Float32, 'motor_speed', 10)
        self.servo_speed_publisher = self.create_publisher(Float32, 'servo_speed', 10)
        self.led_publisher = self.create_publisher(Bool, 'toggle_led', 10)

        # ROS2 Subscriber
        self.led_status_subscriber = self.create_subscription(
            Bool, 'led_status', self.led_status_callback, 10)

        # Timer for publishing movement updates
        timer_period = 0.05  # 20Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Key state manager
        self.key_manager = KeyStateManager(debounce_time=0.02)

        # LED state tracking
        self.led_state = False

        print("=" * 60)
        print("ROS2 Tkinter Publisher Started")
        print("=" * 60)
        print("GUI Controls:")
        print("  WASD keys: Movement control")
        print("  F key: Toggle servo position")
        print("  L key: Toggle LED")
        print("  Sliders: Adjust motor and servo speeds")
        print("  Buttons: Manual control options")
        print("=" * 60)

    def timer_callback(self):
        """Timer callback for publishing movement updates"""
        try:
            # Publish twist message if key state changed
            twist_msg = self.key_manager.get_twist_if_changed()
            if twist_msg is not None:
                self.twist_publisher.publish(twist_msg)
                self.get_logger().info('.2f')

            # Publish servo position if it changed
            servo_position = self.key_manager.get_servo_if_changed()
            if servo_position is not None:
                servo_msg = Int32()
                servo_msg.data = servo_position
                self.servo_publisher.publish(servo_msg)
                self.get_logger().info(f'Publishing servo_position: {servo_position}')
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {e}')

    def publish_twist(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        """Publish twist message"""
        try:
            msg = Twist()
            msg.linear.x = linear_x
            msg.linear.y = linear_y
            msg.linear.z = linear_z
            msg.angular.x = angular_x
            msg.angular.y = angular_y
            msg.angular.z = angular_z
            self.twist_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing Twist message: {e}')

    def publish_servo(self, position):
        """Publish servo position"""
        try:
            msg = Int32()
            msg.data = int(position)
            self.servo_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing servo message: {e}')

    def publish_motor_speed(self, speed):
        """Publish motor speed"""
        try:
            msg = Float32()
            msg.data = float(speed)
            self.motor_speed_publisher.publish(msg)
            self.get_logger().info(f'Publishing motor_speed: {speed}')
        except Exception as e:
            self.get_logger().error(f'Error publishing motor speed: {e}')

    def publish_servo_speed(self, speed):
        """Publish servo speed"""
        try:
            msg = Float32()
            msg.data = float(speed)
            self.servo_speed_publisher.publish(msg)
            self.get_logger().info(f'Publishing servo_speed: {speed}')
        except Exception as e:
            self.get_logger().error(f'Error publishing servo speed message: {e}')

    def publish_led_toggle(self, state):
        """Publish LED toggle"""
        try:
            msg = Bool()
            msg.data = bool(state)
            self.led_publisher.publish(msg)
            self.get_logger().info(f'Publishing LED toggle: {state}')
        except Exception as e:
            self.get_logger().error(f'Error publishing LED toggle message: {e}')

    def led_status_callback(self, msg):
        """Callback for LED status updates"""
        self.led_state = msg.data
        self.get_logger().info(f'Received LED status: {self.led_state}')

    def update_key_state(self, key_code, is_pressed):
        """Update key state from GUI thread"""
        self.key_manager.update_key(key_code, is_pressed)

def main(args=None):
    """Main function - Tkinter runs in main thread, ROS2 in background thread"""
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Bocchi Robot Controller - Tkinter GUI')
    parsed_args, ros_args = parser.parse_known_args(args if args else sys.argv[1:])

    try:
        signal.signal(signal.SIGINT, signal_handler)

        # Create the Tkinter publisher (this starts ROS2 in background thread)
        publisher = TkinterPublisher()

        print("Starting Tkinter GUI main loop...")
        print("Close the Tkinter window to exit")

        # Run Tkinter main loop (this blocks until window is closed)
        publisher.run_gui()

    except KeyboardInterrupt:
        print("\nShutdown requested by user")
    except Exception as e:
        print(f"Error in main: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Shutdown
        if 'publisher' in locals():
            publisher.shutdown()
        print("Application shutdown complete")

if __name__ == '__main__':
    main()