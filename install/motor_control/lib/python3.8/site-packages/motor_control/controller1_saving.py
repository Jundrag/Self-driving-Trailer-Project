import rclpy
from rclpy.node import Node
import serial
import struct
from std_msgs.msg import String, Float32, Int32

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.serial = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        self.get_logger().info(f'serial: {self.serial}')

        self.subscription = self.create_subscription(
            String,
            'motor1_command_topic',
            self.command_callback,
            10
        )

        self.speed_publisher = self.create_publisher(Float32, 'motor_speed', 10)
        self.encoder_14bit_publisher = self.create_publisher(Int32, 'encoder_14bit_position', 10)
        self.encoder_18bit_publisher = self.create_publisher(Int32, 'encoder_18bit_position', 10)

        # Timer for sequential querying
        self.last_command_time = self.get_clock().now()
        self.timer = self.create_timer(1.0, self.timed_operations)  # Check every second
        self.get_logger().info('MotorController node initialized and timer set.')

    def command_callback(self, msg):
        try:
            speed_dps = int(msg.data)
        except ValueError:
            self.get_logger().error(f'Invalid command: {msg.data}')
            return
        command = self.create_motor_speed_command(speed_dps)
        self.send_command(command)
        #self.get_logger().info(f'Command sent to motor1: {msg.data}')
        self.last_command_time = self.get_clock().now()  # Update last command time

    def encode_speed(self, dps_speed):
        encoded_speed = int(dps_speed / 0.01)
        return struct.pack('<i', encoded_speed)

    def calculate_checksum(self, data_bytes):
        return sum(data_bytes) & 0xFF

    def create_motor_speed_command(self, speed_dps):
        frame_head = 0x3E
        cmd_id = 0xA2
        device_id = 0x01
        data_len = 4
        frame_command = bytes([frame_head, cmd_id, device_id, data_len])
        frame_data = self.encode_speed(speed_dps)
        frame_cmd_checksum = self.calculate_checksum([frame_head, cmd_id, device_id, data_len])
        frame_data_checksum = self.calculate_checksum(frame_data)
        return frame_command + bytes([frame_cmd_checksum]) + frame_data + bytes([frame_data_checksum])

    def send_command(self, command):
        command_bytes = bytearray(command)
        self.serial.write(command_bytes)
        response = self.serial.read(10)  # read the response from the motor
        #self.get_logger().info(f'Sent: {command_bytes.hex()} Received: {response.hex()}')
        #self.get_logger().info(f'Raw response: {response}')

    def query_motor_status(self):
        self.get_logger().info('Executing query_motor_status')
        cmd = [0x3E, 0x9C, 0x01, 0x00, 0xDB]
        #print(type(self.calculate_checksum(cmd)))
        #md.append(self.calculate_checksum(cmd))
        self.serial.write(bytearray(cmd))
        response = self.serial.read(10)
        self.get_logger().info(f'Received raw data (len={len(response)}): {response.hex()}')

        if len(response) == 10:
            self.process_response(response)
        else:
            self.get_logger().error(f'Failed to read full response from motor. Received length: {len(response)}.')


    def process_response(self, response):
        self.get_logger().info('Processing response...')
        try:
            self.get_logger().info(f'Full response data: {response.hex()}')

            # Checksum verification
            data_checksum = self.calculate_checksum(response[:-1])
            expected_checksum = response[-1]
            if data_checksum != expected_checksum:
                self.get_logger().error(f'Checksum mismatch: calculated {data_checksum}, expected {expected_checksum}')
                return

            # Motor temperature (int8_t)
            motor_temp = response[0]
            self.get_logger().info(f'Motor temperature: {motor_temp}')

            # Torque current (int16_t)
            torque_current = struct.unpack('<h', response[1:3])[0]
            self.get_logger().info(f'Torque current: {torque_current}')

            # Motor speed (int16_t)
            motor_speed = struct.unpack('<h', response[3:5])[0]
            self.get_logger().info(f'Parsed speed: {motor_speed} dps')

            # Encoder 14-bit position (uint16_t)
            encoder_14bit = struct.unpack('<H', response[5:7])[0] & 0x3FFF
            self.get_logger().info(f'Parsed 14-bit encoder value: {encoder_14bit}')

            # Encoder 18-bit position (uint32_t, taking 3 bytes and masking)
            encoder_18bit = ((response[5] | (response[6] << 8) | (response[7] << 16)) & 0x3FFFF)
            self.get_logger().info(f'Parsed 18-bit encoder value: {encoder_18bit}')

            # Logging and publishing the values
            self.speed_publisher.publish(Float32(data=float(motor_speed)))
            self.encoder_14bit_publisher.publish(Int32(data=encoder_14bit))
            self.encoder_18bit_publisher.publish(Int32(data=encoder_18bit))

            self.get_logger().info(f'Speed: {motor_speed} dps, Encoder 14-bit: {encoder_14bit}, Encoder 18-bit: {encoder_18bit}')
        except Exception as e:
            self.get_logger().error(f'Error in process_response: {e}')


    def timed_operations(self):
        try:
          #  self.get_logger().info('Timer callback triggered.')
            now = self.get_clock().now()
            time_since_last_command = (now - self.last_command_time).nanoseconds / 1e9
            #self.get_logger().info(f'Time since last command: {time_since_last_command} seconds')

            if time_since_last_command > 0.01:  # Example: Query status every 0.1 seconds
                #self.get_logger().info('Time condition met, calling query_motor_status')
                self.query_motor_status()
           # else:
               # self.get_logger().info('Time condition not met, skipping query')
        except Exception as e:
            self.get_logger().error(f'Error in timed_operations: {e}')



def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
