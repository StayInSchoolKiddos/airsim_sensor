#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import re

class CameraInfoPublisher(Node):

    def __init__(self):
        super().__init__('camera_info_publisher')
        
        self.declare_parameter('timer_period', 0.1)
        self.declare_parameter('file_path', '/home/justin/Documents/Unreal Projects/TestUE5/Source/camera1_info.txt')
        self.declare_parameter('info_topic', '/camera_info')
        
        self.timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        self.file_path = self.get_parameter('file_path').get_parameter_value().string_value
        self.info_topic = self.get_parameter('info_topic').get_parameter_value().string_value
        
        self.timer = self.create_timer(self.timer_period, self.publish_camera_info)
        
        self.read_once = False
        # Update this to your file path
        self.camera_info_msg:CameraInfo = self.parse_camera_params
        
        self.publisher_ = self.create_publisher(CameraInfo, self.info_topic, 10)


    def parse_camera_params(self) -> CameraInfo:
        """
        Parse camera parameters from the file and return a CameraInfo message.
        """
        camera_info = CameraInfo()

        try:
            with open(self.file_path, 'r') as file:
                data = file.read()

                # Parse Focal Length
                focal_length_match = re.search(r'Focal Length \(X, Y\): ([\d.]+), ([\d.]+)', data)
                if focal_length_match:
                    fx = float(focal_length_match.group(1))
                    fy = float(focal_length_match.group(2))
                    camera_info.k[0] = fx  # fx
                    camera_info.k[4] = fy  # fy

                # Parse Principal Point
                principal_point_match = re.search(r'Principal Point \(X, Y\): ([\d.]+), ([\d.]+)', data)
                if principal_point_match:
                    cx = float(principal_point_match.group(1))
                    cy = float(principal_point_match.group(2))
                    camera_info.k[2] = cx  # cx
                    camera_info.k[5] = cy  # cy
                
                camera_info.k[8] = 1.0
                
                # Parse Image Resolution
                resolution_match = re.search(r'Image Resolution: Width=(\d+), Height=(\d+)', data)
                if resolution_match:
                    camera_info.width = int(resolution_match.group(1))
                    camera_info.height = int(resolution_match.group(2))

                # Distortion Model
                camera_info.distortion_model = 'plumb_bob'  # Default distortion model, can be set to 'none'

                # Set distortion coefficients to 0 since no distortion is provided
                camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

                # self.get_logger().info("Camera Info parsed successfully!")
                self.read_once = True

        except Exception as e:
            self.get_logger().error(f"Error parsing camera parameters: {e}")

        return camera_info

    def publish_camera_info(self):
        """
        Publish the parsed CameraInfo message.
        """
        if not self.read_once:
            self.camera_info_msg = self.parse_camera_params()
        
        if self.camera_info_msg:
            self.publisher_.publish(self.camera_info_msg)
            # self.get_logger().info('Published CameraInfo message.')

def main(args=None):
    rclpy.init(args=args)

    camera_info_publisher = CameraInfoPublisher()

    rclpy.spin(camera_info_publisher)

    # Shutdown the node when finished
    camera_info_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
