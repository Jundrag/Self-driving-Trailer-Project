import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class ArduinoSerialNode(Node):
    def __init__(self):
        super().__init__('arduino_serial_node')
        self.subscription = self.create_subscription(
            String,
            'actuator_command_topic',
            self.command_callback,
            10)
        self.subscription  # prevent unused variable warning

        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info('Node started and serial port opened')
            time.sleep(3)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")

    def command_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')
        if self.serial_port.is_open:
            time.sleep(0.1) #change
            command_str = (msg.data + '\n').encode()
            self.serial_port.write(command_str)
            self.get_logger().info(f'sent command to actuator: {command_str}')

            # self.serial_port.write(msg.data.encode())
            # to see more debugging process

            response = self.serial_port.readline().decode('utf-8').strip()
            self.get_logger().info(f'Response from actuator: "{response}"')
    
def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
