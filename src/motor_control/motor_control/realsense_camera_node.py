import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import cv2
import numpy as np
import time

class SimpleRealSenseNode(Node):
    def __init__(self):
        super().__init__('simple_realsense_node')
        self.publisher_ = self.create_publisher(Image, 'camera/infrared/image_raw', 10)
        self.real_publisher = self.create_publisher(Image, 'trailer_pose', 10)
        self.bridge = CvBridge()
        
        self.get_logger().info("Attempting to list RealSense devices.")
        
        context = rs.context()
        devices = context.query_devices()
        if len(devices) == 0:
            self.get_logger().error("No RealSense devices were found")
            return
        
        for i, device in enumerate(devices):
            self.get_logger().info(f"Device {i}: {device.get_info(rs.camera_info.name)} - {device.get_info(rs.camera_info.serial_number)}")
        
        self.get_logger().info("Attempting to configure the RealSense pipeline.")
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        try:
            # Configure the pipeline to stream infrared
            self.config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
            self.pipeline.start(self.config)
            self.get_logger().info("RealSense pipeline started successfully.")
        except RuntimeError as e:
            self.get_logger().error(f"Failed to start RealSense pipeline: {e}")
            raise e

        # Set up ArUco dictionary and parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)

        # Load camera calibration data (camera matrix and distortion coefficients)
        self.camera_matrix = np.array([[615.0, 0, 320],
                                       [0, 615.0, 240],
                                       [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.array([0.1, -0.05, 0.001, 0.001, 0], dtype=np.float32)  # Replace with actual distortion coefficients
        
        self.docking_ready = False
        self.docking_start_time = None
        self.countdown_completed = False

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            frames = self.pipeline.wait_for_frames()
            ir_frame = frames.get_infrared_frame()
            if not ir_frame:
                self.get_logger().warning("No infrared frame received.")
                return

            ir_image = np.asanyarray(ir_frame.get_data())
            
            # Detect ArUco markers
            corners, ids, rejected = self.detector.detectMarkers(ir_image)

            # Draw detected markers and print IDs
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(ir_image, corners, ids)
                
                # Estimate pose of each marker and draw axis
                marker_length = 0.05  # Marker size in meters
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, self.camera_matrix, self.dist_coeffs)
                
                if len(ids) >= 2 and not self.countdown_completed:
                    tvec1 = tvecs[0][0]  # Position of first marker in camera coordinate system
                    tvec2 = tvecs[1][0]  # Position of second marker in camera coordinate system

                    # Calculate the Y-axis distance between the two markers and apply scaling factor
                    y_distance = abs(tvec1[1] - tvec2[1]) * 1.5  # Scaling factor applied
                    self.get_logger().info(f"Y-axis distance between markers: {y_distance:.2f} meters")
                    
                    # Determine the message based on the Y-axis distance
                    if y_distance > 0.58:
                        message = "get closer to the target..."
                        self.docking_ready = False
                    else:
                        message = "docking ready!"
                        if not self.docking_ready:
                            self.docking_ready = True
                            self.docking_start_time = time.time()

                    # Display the message on the image with red text
                    mid_point = np.mean(np.concatenate((corners[0][0], corners[1][0])), axis=0).astype(int)
                    cv2.putText(ir_image, f"Distance: {y_distance:.2f}m", tuple(mid_point),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)
                    cv2.putText(ir_image, message, (mid_point[0], mid_point[1] + 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)
                    
                    if self.docking_ready:
                        elapsed_time = time.time() - self.docking_start_time
                        countdown = max(0, 5 - int(elapsed_time))
                        if countdown > 0:
                            cv2.putText(ir_image, f"Counting down: {countdown}", (mid_point[0], mid_point[1] + 60),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)
                        else:
                            self.countdown_completed = True  # Stop showing any text after countdown

                for rvec, tvec in zip(rvecs, tvecs):
                    cv2.drawFrameAxes(ir_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, marker_length)
                    
            # Convert infrared image to BGR format for OpenCV display
            ir_image_bgr = cv2.cvtColor(ir_image, cv2.COLOR_GRAY2BGR)
            
            # Check if GUI support is available for OpenCV
            if hasattr(cv2, 'imshow'):
                # OpenCV processing (display the image)
        
                cv2.imshow('Infrared Frame', ir_image_bgr)
                cv2.waitKey(1)
            else:
                self.get_logger().info("OpenCV built without GUI support. Skipping image display.")

            # Publish the infrared image as a ROS2 Image message
            img_msg = self.bridge.cv2_to_imgmsg(ir_image_bgr, "bgr8")
            self.publisher_.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"Error during frame processing: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleRealSenseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
