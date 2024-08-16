import rclpy	# allow ROS2 fnc & comm in py
from rclpy.node import Node	# import node class to create node
import serial	# pyserial library for serial comm
import time
import struct

from std_msgs.msg import String #new

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.serial = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)  #motor_right
        self.get_logger().info(f'serial: {self.serial}')
        #new"
        self.subscription = self.create_subscription(
            String,
            'motor1_command_topic',
            self.command_callback,
            10
        )
        #new"
        print(f'serial: {self.serial}')

        #self.initialize_motor()

   #new  
    def command_callback(self, msg):
        
        # self.serial.write(msg.data.encode())
        # self.get_logger().info(f'Command sent to motor2: {msg.data}')

        #new"
        try:
            speed_dps = int(msg.data)
        except ValueError:
            self.get_logger().error(f'Invalid command: {msg.data}')
            return
        command = self.create_motor_speed_command(speed_dps)
        self.send_command(command)
        #self.get_logger().info(f'Command sent to motor2: {msg.data}')
        #new"

    def encode_speed(self, dps_speed):
    # convert degrees per second to the int32_t representation considering 0.01 dps/LSB
        encoded_speed = int(dps_speed / 0.01)  # convert the speed to the LSB unit used by the system
        # Pack as little-endian int32
        speed_bytes = struct.pack('<i', encoded_speed)  # '<i' denotes a little-endian signed int
        return speed_bytes

    def calculate_checksum(self, data_bytes):
        return sum(data_bytes) & 0xFF

    def create_motor_speed_command(self, speed_dps):
    # frame command
        frame_head = 0x3E
        cmd_id = 0xA2   # A2 = closed-loop speed control mode
        device_id = 0x01    # motor ID
        data_len = 4  # Set as constant 4 bytes as the example provided shows a fixed format
        
        frame_command = bytes([frame_head, cmd_id, device_id, data_len])   # type: bytes
        frame_data = self.encode_speed(speed_dps) # little-endian int32 packed speed; type: bytes


        frame_cmd_checksum = self.calculate_checksum(bytes([frame_head, cmd_id, device_id, data_len]))    # type: int
        frame_data_checksum = self.calculate_checksum(frame_data)
        
        full_command = frame_command + bytes([frame_cmd_checksum]) + frame_data + bytes([frame_data_checksum])

        return full_command


    # def initialize_motor(self):
    #     #self.send_command([0x3E, 0x14, 0x01, 0x00, 0x53]) # default read setting
    #     time.sleep(13)
    #     self.send_command(self.create_motor_speed_command(1500)) # turn motor on
    #     time.sleep(2)
    #     self.send_command(self.create_motor_speed_command(2500)) 
    #     time.sleep(2)
    #     self.send_command(self.create_motor_speed_command(3000)) 
    #     time.sleep(2)
    #     self.send_command([0x3E, 0x80, 0x01, 0x00, 0xBF]) # Turn motor off

    def send_command(self, command):
        command_bytes = bytearray(command)	#bytearray(list): returns array of given bytes
        self.serial.write(command_bytes)
        response = self.serial.read(10)  # read the response from the motor
       # self.get_logger().info(f'Sent: {command_bytes.hex()} Received: {response.hex()}')


def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

