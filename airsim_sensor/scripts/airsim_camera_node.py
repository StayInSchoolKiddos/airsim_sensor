#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('ue_image_subscriber')

        # Create a subscriber to the 'ue_image_topic'
        self.subscription = self.create_subscription(
            Image,
            'ue_image_data',
            self.listener_callback,
            10)

        # Initialize CvBridge for converting ROS images to OpenCV format
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        self.get_logger().info('Received image data')

        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Display the image using OpenCV
        cv2.imshow("Unreal Engine Image", cv_image)
        cv2.waitKey(1)  # Display image for 1ms, then refresh


def main(args=None):
    rclpy.init(args=args)

    # Create the image subscriber node
    image_subscriber = ImageSubscriber()

    try:
        # Spin the node so it keeps running
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        # Gracefully handle shutdown
        pass
    finally:
        # Cleanup when the node is stopped
        image_subscriber.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
