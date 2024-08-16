import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy

class SequenceController(Node):
    def __init__(self):
        super().__init__('sequence_controller')
        self.actuator_publisher = self.create_publisher(String, 'linear_actuator_cmd', 10)
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.current_command = None  # To keep track of the current command
        self.previous_button_state = {'L1': False, 'R1': False}

    def joy_callback(self, msg):
        # Assuming L1 is button[4] and R1 is button[5]
        l1_pressed = msg.buttons[4]
        r1_pressed = msg.buttons[5]

        if l1_pressed and not self.previous_button_state['L1']:
            self.get_logger().info('L1 button pressed')
            self.send_actuator_command('0')  # Extend command
        elif r1_pressed and not self.previous_button_state['R1']:
            self.get_logger().info('R1 button pressed')
            self.send_actuator_command('1')  # Retract command

        # Update the previous button state
        self.previous_button_state['L1'] = l1_pressed
        self.previous_button_state['R1'] = r1_pressed

    def send_actuator_command(self, command):
        # Always send the command regardless of the current command
        msg = String()
        msg.data = command
        self.actuator_publisher.publish(msg)
        self.get_logger().info(f'Sent actuator command: {msg.data}')
        self.current_command = command

def main(args=None):
    rclpy.init(args=args)
    sequence_controller = SequenceController()
    rclpy.spin(sequence_controller)
    sequence_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
