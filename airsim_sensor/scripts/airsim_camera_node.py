#!/usr/bin/env python3

import airsim
import numpy as np
import os
import pprint
import time
import cv2
from concurrent.futures import ThreadPoolExecutor
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time 
pp = pprint.PrettyPrinter(indent=4)

# Get WSL_HOST_IP
WSL_HOST_IP = os.getenv('WSL_HOST_IP', 'localhost')
print('WSL_HOST_IP: ', WSL_HOST_IP)

client = airsim.VehicleClient(ip=WSL_HOST_IP)
client.confirmConnection()

# API to access camera data
print('Connected to AirSim')
front_camera = 'front_center_custom'
camera_info = client.simGetCameraInfo(front_camera)
print("CameraInfo:")
pp.pprint(camera_info)

# Function to process image
def process_image(image):
    img1d = np.frombuffer(image.image_data_uint8, dtype=np.uint8)
    img_rgb = img1d.reshape(image.height, image.width, 3)
    img_rgba = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2RGBA)
    return img_rgba

# ROS 2 Node to publish image
class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'airsim_image', 1)
        self.bridge = CvBridge()
        self.client = client
        self.front_camera = front_camera
        self.dt = 0.0001
        self.timer = self.create_timer(self.dt, self.timer_callback)  # Increase rate by reducing timer period

    def timer_callback(self):
        start_time = time.time()
        responses = self.client.simGetImages([
            airsim.ImageRequest(self.front_camera, airsim.ImageType.Scene, False, 
                                compress=False)
        ])
        print("Time taken to get image: ", time.time() - start_time)
        if responses:
            image = responses[0]
            img_rgb = process_image(image)
            #switch image to bgr format
            img_rgb = cv2.cvtColor(img_rgb, cv2.COLOR_RGBA2BGRA)
            ros_image = self.bridge.cv2_to_imgmsg(img_rgb, encoding="rgba8")
            self.publisher_.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
