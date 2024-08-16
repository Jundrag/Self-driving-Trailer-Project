import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy

class JoystickControl(Node):
    def __init__(self):
        super().__init__('joystick_control')
        self.motor1_publisher = self.create_publisher(String, 'motor1_command_topic', 10)   # publish str to motor1_command_topic
        self.motor2_publisher = self.create_publisher(String, 'motor2_command_topic', 10)
        self.subscription = self.create_subscription(   # subscribes to 'joy' topic expecting Joy msg topic. msg received > joy_callback is executed
            Joy,
            'joy',
            self.joy_callback,
            10
        )

    def joy_callback(self, msg):
        # Example: assuming the joystick's left stick vertical axis (index 1) controls speed
        axis_value_left = msg.axes[4]
        axis_value_right = msg.axes[1]  # Assuming right stick controls motor2, for example
        speed_dps_motor1 = int(axis_value_left * -2000)  # Scale joystick input to motor speed range
        speed_dps_motor2 = int(axis_value_right * 5000)  # Scale joystick input to motor speed range
        self.motor1_publisher.publish(String(data=str(speed_dps_motor1)))
        self.motor2_publisher.publish(String(data=str(speed_dps_motor2)))
        #self.get_logger().info(f'Motor1 command: {speed_dps_motor1} dps, Motor2 command: {speed_dps_motor2} dps')

def main(args=None):
    rclpy.init(args=args)
    joystick_control = JoystickControl()
    rclpy.spin(joystick_control)
    joystick_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()