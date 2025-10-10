import rclpy
from rclpy.node import Node
import signal
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool, Float32
import sys
import argparse
import threading
import time
import wx
from wx.lib.newevent import NewEvent

# Custom event for ROS2 publishing from GUI thread
PublishEvent, EVT_PUBLISH = NewEvent()

minimal_publisher = None

def signal_handler(sig, frame):
    print("\nSIGINT (Ctrl+C) received! Performing graceful shutdown.")
    if minimal_publisher:
        minimal_publisher.destroy_node()
    rclpy.shutdown()
    wx.CallAfter(wx.GetApp().ExitMainLoop)
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
        self.led_changed = False
        self.led_state = False

    def update_key(self, key_code, is_pressed=True):
        current_time = time.time()

        # Handle F key for servo toggle
        if key_code == 70 and is_pressed:  # F key
            self.servo_position = 180 if self.servo_position == 0 else 0
            self.servo_changed = True
            self.last_key_time = current_time
            return True

        # Handle L key for LED toggle
        if key_code == 76 and is_pressed:  # L key
            self.led_state = not self.led_state
            self.led_changed = True
            self.last_key_time = current_time
            return True

        if is_pressed:
            if key_code not in self.current_keys:
                self.current_keys.add(key_code)
                self.last_key_time = current_time
                self.key_changed = True
                return True
        else:
            if key_code in self.current_keys:
                self.current_keys.remove(key_code)
                self.last_key_time = current_time
                self.key_changed = True
                return True
        return False

    def get_twist_if_changed(self):
        if self.key_changed:
            self.key_changed = False
            return self._calculate_twist()
        return None

    def _calculate_twist(self):
        """Calculate Twist message based on current pressed keys"""
        twist = Twist()

        # Key mappings based on WASD keys
        # W = forward (87)
        # S = backward (83)
        # A = turn left (65)
        # D = turn right (68)

        # Linear velocity (forward/backward)
        if 87 in self.current_keys:  # W
            twist.linear.x = 0.5
        elif 83 in self.current_keys:  # S
            twist.linear.x = -0.5
        else:
            twist.linear.x = 0.0

        # Angular velocity (left/right turn)
        if 65 in self.current_keys:  # A
            twist.angular.z = 0.5
        elif 68 in self.current_keys:  # D
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
        if self.servo_changed:
            self.servo_changed = False
            return self.servo_position
        return None

    def get_led_if_changed(self):
        if self.led_changed:
            self.led_changed = False
            return self.led_state
        return None

class WxPublisherFrame(wx.Frame):
    """wxPython GUI frame for robot control"""

    def __init__(self, parent, publisher_node):
        super().__init__(parent, title="Bocchi Robot Controller - wxPython", size=(400, 300))

        self.publisher_node = publisher_node
        self.key_manager = publisher_node.key_manager

        # Set up the GUI
        self.init_ui()

        # Bind keyboard events
        self.Bind(wx.EVT_KEY_DOWN, self.on_key_down)
        self.Bind(wx.EVT_KEY_UP, self.on_key_up)
        self.Bind(EVT_PUBLISH, self.on_publish_event)

        # Set focus to capture keyboard events
        self.SetFocus()

        # Start timer for publishing updates
        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_timer, self.timer)
        self.timer.Start(50)  # 50ms timer

        self.Show()

    def init_ui(self):
        """Initialize the user interface"""
        panel = wx.Panel(self)
        vbox = wx.BoxSizer(wx.VERTICAL)

        # Title
        title = wx.StaticText(panel, label="Bocchi Robot Controller")
        font = title.GetFont()
        font.SetPointSize(14)
        font.SetWeight(wx.FONTWEIGHT_BOLD)
        title.SetFont(font)
        vbox.Add(title, flag=wx.ALIGN_CENTER|wx.TOP|wx.BOTTOM, border=10)

        # Instructions
        instructions = wx.StaticText(panel, label="Use WASD keys for movement\nF key for servo toggle\nL key for LED toggle")
        vbox.Add(instructions, flag=wx.ALIGN_CENTER|wx.BOTTOM, border=10)

        # Status display
        self.status_text = wx.StaticText(panel, label="Ready")
        vbox.Add(self.status_text, flag=wx.ALIGN_CENTER|wx.BOTTOM, border=10)

        # Key state display
        self.key_state_text = wx.StaticText(panel, label="Keys pressed: None")
        vbox.Add(self.key_state_text, flag=wx.ALIGN_CENTER|wx.BOTTOM, border=10)

        # Servo status
        self.servo_text = wx.StaticText(panel, label="Servo position: 0°")
        vbox.Add(self.servo_text, flag=wx.ALIGN_CENTER|wx.BOTTOM, border=10)

        # LED status
        self.led_text = wx.StaticText(panel, label="LED: OFF")
        vbox.Add(self.led_text, flag=wx.ALIGN_CENTER|wx.BOTTOM, border=10)

        panel.SetSizer(vbox)

    def on_key_down(self, event):
        """Handle key press events"""
        key_code = event.GetKeyCode()
        if self.key_manager.update_key(key_code, True):
            self.update_display()
            # Trigger immediate publish
            wx.PostEvent(self, PublishEvent())

    def on_key_up(self, event):
        """Handle key release events"""
        key_code = event.GetKeyCode()
        if self.key_manager.update_key(key_code, False):
            self.update_display()
            # Trigger immediate publish
            wx.PostEvent(self, PublishEvent())

    def on_publish_event(self, event):
        """Handle publish events from key presses"""
        self.publisher_node.publish_updates()

    def on_timer(self, event):
        """Timer callback for regular updates"""
        self.publisher_node.publish_updates()
        self.update_display()

    def update_display(self):
        """Update the GUI display with current state"""
        # Update key state display
        keys_pressed = []
        key_names = {65: 'A', 68: 'D', 83: 'S', 87: 'W', 70: 'F', 76: 'L'}
        for key_code in self.key_manager.current_keys:
            keys_pressed.append(key_names.get(key_code, str(key_code)))

        if keys_pressed:
            self.key_state_text.SetLabel(f"Keys pressed: {', '.join(keys_pressed)}")
        else:
            self.key_state_text.SetLabel("Keys pressed: None")

        # Update servo position
        self.servo_text.SetLabel(f"Servo position: {self.key_manager.servo_position}°")

        # Update LED status
        led_status = "ON" if self.key_manager.led_state else "OFF"
        self.led_text.SetLabel(f"LED: {led_status}")

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher_wx')
        self.twist_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.servo_publisher = self.create_publisher(Int32, 'servo_position', 10)
        self.motor_speed_publisher = self.create_publisher(Float32, 'motor_speed', 10)
        self.servo_speed_publisher = self.create_publisher(Float32, 'servo_speed', 10)
        self.led_publisher = self.create_publisher(Bool, 'toggle_led', 10)
        self.led_status_subscriber = self.create_subscription(Bool, 'led_status', self.led_status_callback, 10)

        # Initialize key state manager
        self.key_manager = KeyStateManager(debounce_time=0.02)
        self.led_state = False

        print("=" * 50)
        print("ROS2 wxPython Keyboard Interface Started")
        print("=" * 50)
        print("Controls:")
        print("  WASD keys for movement")
        print("  F key for servo toggle")
        print("  L key for LED toggle")
        print("=" * 50)

    def publish_updates(self):
        """Publish all pending updates"""
        # Publish movement
        twist_msg = self.key_manager.get_twist_if_changed()
        if twist_msg is not None:
            self.twist_publisher.publish(twist_msg)
            self.get_logger().info(f'Publishing cmd_vel: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}')

        # Publish servo position
        servo_position = self.key_manager.get_servo_if_changed()
        if servo_position is not None:
            servo_msg = Int32()
            servo_msg.data = servo_position
            self.servo_publisher.publish(servo_msg)
            self.get_logger().info(f'Publishing servo position: {servo_position}°')

        # Publish LED toggle
        led_state = self.key_manager.get_led_if_changed()
        if led_state is not None:
            led_msg = Bool()
            led_msg.data = led_state
            self.led_publisher.publish(led_msg)
            self.get_logger().info(f'Publishing LED toggle: {led_state}')

    def led_status_callback(self, msg):
        """Callback for LED status updates"""
        self.led_state = msg.data
        print(f"Received LED status: {self.led_state}")

def ros2_spin_thread(node):
    """Run ROS2 spin in a separate thread"""
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"ROS2 spin error: {e}")

def main(args=None):
    global minimal_publisher

    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Bocchi Robot Controller - wxPython GUI')
    parsed_args, ros_args = parser.parse_known_args(args if args else sys.argv[1:])

    try:
        rclpy.init(args=ros_args)
        signal.signal(signal.SIGINT, signal_handler)

        minimal_publisher = MinimalPublisher()

        # Start ROS2 spin in a separate thread
        ros_thread = threading.Thread(target=ros2_spin_thread, args=(minimal_publisher,), daemon=True)
        ros_thread.start()

        # Start wxPython GUI
        app = wx.App()
        WxPublisherFrame(None, minimal_publisher)

        print("Starting wxPython GUI...")
        app.MainLoop()

    except KeyboardInterrupt:
        print("\nShutdown requested by user")
    except Exception as e:
        print(f"Error in main: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Destroy the node explicitly
        if minimal_publisher:
            try:
                minimal_publisher.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        print("ROS2 node shutdown complete")

if __name__ == '__main__':
    main()