import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32

class MotorStatusMonitor(Node):
    def __init__(self):
        super().__init__('motor_status_monitor')

        # Subscriptions to motor speed and encoder positions
        self.create_subscription(Float32, 'motor_speed', self.speed_callback, 10)
        self.create_subscription(Int32, 'encoder_14bit_position', self.encoder_14bit_callback, 10)
        self.create_subscription(Int32, 'encoder_18bit_position', self.encoder_18bit_callback, 10)


    def speed_callback(self, msg):
        self.get_logger().info(f'Received motor speed: {msg.data} dps')

    def encoder_14bit_callback(self, msg):
        # Ensuring value is within 14-bit range
        encoder_value = msg.data & 0x3FFF
        self.get_logger().info(f'Received 14-bit encoder position (reducer): {encoder_value}')

    def encoder_18bit_callback(self, msg):
        # Since we're interested in the 18-bit value, ensure correct handling
        encoder_value = msg.data & 0x3FFFF
        self.get_logger().info(f'Received 18-bit encoder position (motor): {encoder_value}')

def main(args=None):
    rclpy.init(args=args)
    motor_status_monitor = MotorStatusMonitor()
    rclpy.spin(motor_status_monitor)
    motor_status_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
