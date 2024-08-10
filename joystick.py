import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import time

class JoystickControl(Node):
    def __init__(self):
        super().__init__('joystick_control')
        self.motor1_publisher = self.create_publisher(String, 'motor1_speed_cmd', 10)
        self.motor2_publisher = self.create_publisher(String, 'motor2_speed_cmd', 10)

        self.motor1_angle_publisher = self.create_publisher(String, 'motor1_angle_cmd', 10)
        self.motor2_angle_publisher = self.create_publisher(String, 'motor2_angle_cmd', 10)

        self.motor1_speed_limit_publisher = self.create_publisher(String, 'motor1_speed_limit_cmd', 10)
        self.motor2_speed_limit_publisher = self.create_publisher(String, 'motor2_speed_limit_cmd', 10)

        self.mode_publisher = self.create_publisher(String, 'control_mode', 10)

        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        self.motor1_angle = 0
        self.motor1_speed_limit = 100
        self.motor2_angle = 0
        self.motor2_speed_limit = 100
        self.control_mode = "angle"
        self.last_mode_switch_time = 0.0 # Initialize this attribute

    def joy_callback(self, msg):
        angle_increment = 50
        speed_increment = 100

        # Detect mode switch
        middle_button_pressed = msg.buttons[10]  # assuming button[10] is the middle button
        current_time = time.time()
        if middle_button_pressed and (current_time - self.last_mode_switch_time) > 1.0:  # Debounce time increased to 1 second
            self.last_mode_switch_time = current_time
            if self.control_mode == "speed":
                self.control_mode = "angle"
            else:
                self.control_mode = "speed"
            self.get_logger().info(f"Switched control mode to {self.control_mode}")
            self.mode_publisher.publish(String(data=self.control_mode))
            time.sleep(0.3)  # debounce delay to avoid rapid switching

        # motor1 control
        if self.control_mode == "speed":
            axis_value_speed_motor1 = msg.axes[4]  # rotation controller
            self.speed_dps_motor1 = int(axis_value_speed_motor1 * -1000)  # Scale joystick input to motor speed range
            motor1_speed_command = f"{self.speed_dps_motor1}"
            self.motor1_publisher.publish(String(data=motor1_speed_command))

        elif self.control_mode == "angle":
            axis_value_angle_motor2 = msg.axes[7]  # D-pad up/down
            axis_value_speed_limit_motor2 = msg.axes[6]  # D-pad left/right

            if axis_value_angle_motor2 < -0.5:  # D-pad up pressed
                self.motor2_angle -= angle_increment
                self.get_logger().info(f'D-Pad Up: Decrease motor1 angle to {self.motor2_angle}')

            elif axis_value_angle_motor2 > 0.5:  # D-pad down pressed
                self.motor2_angle += angle_increment
                self.get_logger().info(f'D-Pad Down: Increase motor1 angle to {self.motor2_angle}')

            if axis_value_speed_limit_motor2 < -0.5:  # D-pad right pressed
                self.motor2_speed_limit += speed_increment
                self.get_logger().info(f'D-Pad Right: Increase motor1 speed limit to {self.motor2_speed_limit} dps')

            elif axis_value_speed_limit_motor2 > 0.5:  # D-pad left pressed
                self.motor2_speed_limit -= speed_increment
                self.get_logger().info(f'D-Pad Left: Decrease motor1 speed limit to {self.motor2_speed_limit} dps')

            motor2_angle_command = f"{self.motor2_angle}"
            motor2_speed_limit_command = f"{self.motor2_speed_limit}"

            self.motor2_angle_publisher.publish(String(data=motor2_angle_command))
            self.motor2_speed_limit_publisher.publish(String(data=motor2_speed_limit_command))

        # Motor2 control
        if self.control_mode == "speed":
            axis_value_speed_motor2 = msg.axes[1]  # Assuming right stick controls motor2
            self.speed_dps_motor2 = int(axis_value_speed_motor2 * 1000)  # Scale joystick input to motor speed range
            motor2_speed_command = f"{self.speed_dps_motor2}"
            self.motor2_publisher.publish(String(data=motor2_speed_command))

        elif self.control_mode == "angle":
            if msg.buttons[2]:  # Triangle button pressed
                self.motor1_angle += angle_increment
                self.get_logger().info(f'Triangle button: Increase motor2 angle to {self.motor1_angle}')

            if msg.buttons[0]:  # Cross button pressed
                self.motor1_angle -= angle_increment
                self.get_logger().info(f'Cross button: Decrease motor2 angle to {self.motor1_angle}')

            if msg.buttons[1]:  # Circle button pressed
                self.motor1_speed_limit += speed_increment
                self.get_logger().info(f'Circle button: Increase motor2 speed limit to {self.motor1_speed_limit} dps')

            if msg.buttons[3]:  # Square button pressed
                self.motor1_speed_limit -= speed_increment
                self.get_logger().info(f'Square button: Decrease motor2 speed limit to {self.motor1_speed_limit} dps')

            motor1_angle_command = f"{-self.motor1_angle}"
            motor1_speed_limit_command = f"{self.motor1_speed_limit}"

            self.motor1_angle_publisher.publish(String(data=motor1_angle_command))
            self.motor1_speed_limit_publisher.publish(String(data=motor1_speed_limit_command))

def main(args=None):
    rclpy.init(args=args)
    joystick_control = JoystickControl()
    rclpy.spin(joystick_control)
    joystick_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
