import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.duration import Duration
import time

class SequenceController(Node):
    def __init__(self):
        super().__init__('sequence_controller')
        self.actuator_publisher = self.create_publisher(String, 'actuator_command_topic', 10)
        self.motor1_publisher = self.create_publisher(String, 'motor1_command_topic', 10)
        self.motor2_publisher = self.create_publisher(String, 'motor2_command_topic', 10)
        self.timer = self.create_timer(1.0, self.execute_sequence)
        self.sequence_step = 0
        self.start_time = self.get_clock().now()
        self.get_logger().info('tankturn initiallized')

    def execute_sequence(self):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds/1e9

        if self.sequence_step == 0:
            # Step 1: Extend actuator for 10 seconds
            self.send_actuator_command('1')  # Extend command 1: retract, 0: extend
            self.get_logger().info('Extending actuator for 10 seconds')
            self.start_time = self.get_clock().now()
            self.sequence_step += 1

        elif self.sequence_step == 1 and elapsed_time >= 12:
            self.send_motor_command('300')  # Start motors at 1500 dps
            self.get_logger().info('Running motors for 5 seconds L')
            self.start_time = self.get_clock().now()
            self.sequence_step += 1

        elif self.sequence_step == 2 and elapsed_time >= 5: # 300 3s
            self.send_motor_command('0')
            self.start_time = self.get_clock().now()
            self.sequence_step += 1

        elif self.sequence_step == 3 and elapsed_time >= 2: # stop 2s
            self.send_motor_command('-300')
            self.get_logger().info('Running motors for 10 seconds R')
            self.start_time = self.get_clock().now()
            self.sequence_step += 1

        elif self.sequence_step == 4 and elapsed_time >= 10: # -300 6s
            self.send_motor_command('0')
            self.start_time = self.get_clock().now()
            self.sequence_step += 1

        elif self.sequence_step == 5 and elapsed_time >= 2: # stop 2s
            self.send_motor_command('300')
            self.get_logger().info('Running motors for 5 seconds L')
            self.start_time = self.get_clock().now()
            self.sequence_step += 1

        elif self.sequence_step == 6 and elapsed_time >= 5: # 300 3s
            self.send_motor_command('0')            
            self.start_time = self.get_clock().now()
            self.sequence_step += 1 

        elif self.sequence_step == 7 and elapsed_time >= 2: # stop 2s
            self.send_actuator_command('0')  # Retract command
            self.get_logger().info('Stopping motors and retracting actuator')
            self.start_time = self.get_clock().now()
            self.sequence_step += 1


        elif self.sequence_step == 8 and elapsed_time >= 2:
            self.get_logger().info('Sequence complete')
            self.sequence_step += 1

    def send_actuator_command(self, command):
        msg = String()
        msg.data = command
        self.actuator_publisher.publish(msg)
        self.get_logger().info(f'Sent actuator command: {msg.data}')

    def send_motor_command(self, command):
        msg = String()
        msg.data = command
        self.motor1_publisher.publish(msg)
        self.motor2_publisher.publish(msg)
        self.get_logger().info(f'Sent motor command: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    sequence_controller = SequenceController()
    rclpy.spin(sequence_controller)
    sequence_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
