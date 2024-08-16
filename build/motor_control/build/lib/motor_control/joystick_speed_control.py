
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy

class JoystickControl(Node):
    def __init__(self):
        super().__init__('joystick_control')
        self.motor1_publisher = self.create_publisher(String, 'motor1_command_topic', 10)   # publish str to motor1_command_topic
        self.motor2_publisher = self.create_publisher(String, 'motor2_command_topic', 10)

        self.motor1_angle_publisher = self.create_publisher(String, 'motor1_angle_command_topic', 10)
        self.motor2_angle_publisher = self.create_publisher(String, 'motor2_angle_command_topic', 10)

        self.motor1_speed_limit_publisher = self.create_publisher(String, 'motor1_speed_limit_command_topic', 10)
        self.motor2_speed_limit_publisher = self.create_publisher(String, 'motor2_speed_limit_command_topic', 10)

        self.subscription = self.create_subscription(   # subscribes to 'joy' topic expecting Joy msg topic. msg received > joy_callback is executed
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        self.motor1_angle = 0
        self.motor1_speed_limit = 100
        self.motor2_angle = 0
        self.motor2_speed_limit = 100

    def joy_callback(self, msg):
        
        angle_increment = 360 # angle change increment in deg
        speed_increment = 100 # speed limit change increment in dps 


        # motor1 control
        axis_value_speed_motor1 = msg.axes[4] # rotation controller


        axis_value_angle_motor1 = msg.axes[7] # D-pad up/down
        axis_value_speed_limit_motor1 = msg.axes[6] # D-pad left/right

        if axis_value_angle_motor1 > 0.5:  # D-pad up pressed
            self.motor1_angle += angle_increment
            self.get_logger().info(f'D-Pad Up: Increase motor1 angle to {self.motor1_angle}')

        elif axis_value_angle_motor1 < -0.5:  # D-pad down pressed
            self.motor1_angle -= angle_increment
            self.get_logger().info(f'D-Pad Down: Decrease motor1 angle to {self.motor1_angle}')

        if axis_value_speed_limit_motor1 < -0.5:  # D-pad right pressed
            self.motor1_speed_limit += speed_increment
            self.get_logger().info(f'D-Pad Right: Increase motor1 speed limit to {self.motor1_speed_limit} dps')

        elif axis_value_speed_limit_motor1 > 0.5:  # D-pad left pressed
            self.motor1_speed_limit -= speed_increment
            self.get_logger().info(f'D-Pad Left: Decrease motor1 speed limit to {self.motor1_speed_limit} dps')

        # motor2 control

        axis_value_speed_motor2 = msg.axes[1]  # Assuming right stick controls motor2, for example

        if msg.buttons[2]:  # Triangle button pressed
            self.motor2_angle += angle_increment
            self.get_logger().info(f'Triangle button: Increase motor2 angle to {self.motor2_angle}')

        if msg.buttons[0]:  # Cross button pressed
            self.motor2_angle -= angle_increment
            self.get_logger().info(f'Cross button: Decrease motor2 angle to {self.motor2_angle}')

        if msg.buttons[1]:  # Circle button pressed
            self.motor2_speed_limit += speed_increment 
            self.get_logger().info(f'Square button: Increase motor2 speed limit to {self.motor2_speed_limit} dps')
        if msg.buttons[3]:  # Square button pressed
            self.motor2_speed_limit -= speed_increment
            self.get_logger().info(f'Circle button: Decrease motor2 speed limit to {self.motor2_speed_limit} dps')

        # speed calibration
        self.speed_dps_motor1 = int(axis_value_speed_motor1 * -2000)  # Scale joystick input to motor speed range
        self.speed_dps_motor2 = int(axis_value_speed_motor2 * 1000)  # Scale joystick input to motor speed range

        # publishing 6 topics: motor 1 & 2 speed,angle,speed lim 

        motor1_speed_command = f"{self.speed_dps_motor1}"
        motor2_speed_command = f"{self.speed_dps_motor2}"

        motor1_angle_command = f"{self.motor1_angle}"
        motor2_angle_command = f"{self.motor2_angle}"

        
        motor1_speed_limit_command = f"{self.motor1_speed_limit}"
        motor2_speed_limit_command = f"{self.motor2_speed_limit}"

    
        self.motor1_publisher.publish(String(data=motor1_speed_command))
        self.motor1_angle_publisher.publish(String(data=motor1_angle_command))
        self.motor1_speed_limit_publisher.publish(String(data=motor1_speed_limit_command))

        self.motor2_publisher.publish(String(data=motor2_speed_command))
        self.motor2_angle_publisher.publish(String(data=motor2_angle_command))
        self.motor2_speed_limit_publisher.publish(String(data=motor2_speed_limit_command))

def main(args=None):
    rclpy.init(args=args)
    joystick_control = JoystickControl()
    rclpy.spin(joystick_control)
    joystick_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()