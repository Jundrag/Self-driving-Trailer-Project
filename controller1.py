import rclpy
from rclpy.node import Node
import serial
import struct
from std_msgs.msg import String

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.serial = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)  # motor right
        self.get_logger().info(f'serial: {self.serial}')

        self.subscription = self.create_subscription(
            String,
            'motor1_command_topic',
            self.command_callback,
            10
        )
        self.subscription2 = self.create_subscription(
            String,
            'motor1_angle_command_topic',
            self.angle_callback,
            10
        )
        self.subscription3 = self.create_subscription(
            String,
            'motor1_speed_limit_command_topic',
            self.speed_limit_callback,
            10
        )
        self.subscription4 = self.create_subscription(
            String,
            'control_mode_topic',
            self.mode_callback,
            10
        )

        self.angle_d = 0
        self.speed_dps = 0
        self.control_mode = "angle"

    def mode_callback(self, msg):
        self.control_mode = msg.data
        self.get_logger().info(f"Control mode changed to: {self.control_mode}")

    def command_callback(self, msg):
        if self.control_mode == "speed":
            try:
                speed_dps = int(msg.data)
                command = self.create_motor_speed_command(speed_dps)
                self.send_command(command)
            except ValueError:
                self.get_logger().error(f'Invalid command: {msg.data}')

    def angle_callback(self, msg):
        if self.control_mode == "angle":
            try:
                self.angle_d = int(msg.data)
                self.send_motor_angle_command()
            except ValueError:
                self.get_logger().error(f'Invalid angle command: {msg.data}')

    def speed_limit_callback(self, msg):
        if self.control_mode == "angle":
            try:
                self.speed_dps = int(msg.data)
                self.send_motor_angle_command()
            except ValueError:
                self.get_logger().error(f'Invalid speed limit command: {msg.data}')

    def encode_speed(self, dps_speed):
        encoded_speed = int(dps_speed / 0.01)
        speed_bytes = struct.pack('<i', encoded_speed)
        return speed_bytes

    def encode_angle(self, angle):
        encoded_angle = int(angle * 100)
        angle_bytes = struct.pack('<q', encoded_angle)
        return angle_bytes

    def encode_speed_limit(self, speed):
        if speed < 0:
            speed = 0
        encoded_speed = int(speed * 100)
        speed_bytes = struct.pack('<I', encoded_speed)
        return speed_bytes

    def calculate_checksum(self, data_bytes):
        return sum(data_bytes) & 0xFF

    def create_motor_speed_command(self, speed_dps):
        frame_head = 0x3E
        cmd_id = 0xA2
        device_id = 0x01
        data_len = 4
        frame_command = bytes([frame_head, cmd_id, device_id, data_len])
        frame_data = self.encode_speed(speed_dps)
        frame_cmd_checksum = self.calculate_checksum(frame_command)
        frame_data_checksum = self.calculate_checksum(frame_data)
        full_command = frame_command + bytes([frame_cmd_checksum]) + frame_data + bytes([frame_data_checksum])
        return full_command

    def create_motor_angle_command(self):
        frame_head = 0x3E
        cmd_id = 0xA4
        device_id = 0x01
        data_len = 0x0C
        frame_command = bytes([frame_head, cmd_id, device_id, data_len])
        angle_bytes = self.encode_angle(self.angle_d)
        speed_bytes = self.encode_speed_limit(self.speed_dps)
        frame_data = angle_bytes + speed_bytes
        frame_cmd_checksum = self.calculate_checksum(frame_command)
        frame_data_checksum = self.calculate_checksum(frame_data)
        full_command = frame_command + bytes([frame_cmd_checksum]) + frame_data + bytes([frame_data_checksum])
        return full_command

    def send_command(self, command):
        command_bytes = bytearray(command)
        self.serial.write(command_bytes)
        response = self.serial.read(10)

    def send_motor_angle_command(self):
        command = self.create_motor_angle_command()
        self.send_command(command)

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
