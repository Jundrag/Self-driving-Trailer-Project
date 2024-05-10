import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import UInt16

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.publisher = self.create_publisher(UInt16, 'actuator_status', 10)
        self.subscription = self.create_subscription(
            UInt16,
            'actuator_commands',
            self.command_callback,
            10)
        self.subscription  # prevent unused variable warning

    def command_callback(self, msg):
        command = str(msg.data) + '\n'
        self.serial_port.write(command.encode('utf-8'))
        status = self.serial_port.readline().decode('utf-8').strip()
        if status:
            self.publisher.publish(UInt16(data=int(status)))

def main(args=None):
    rclpy.init(args=args)
    serial_bridge = SerialBridge()
    try:
        rclpy.spin(serial_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        serial_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
