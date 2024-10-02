#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import airsim
import os
import tf2_geometry_msgs
import tf_transformations

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener



class AirsimCameraFrame(Node):    
    """
    Attributes:
    - User defines the location and rotation of the AirsimCameraFrame
    - User defines the name of the AirsimCameraFrame
    - User defines the name of the AirsimMovePawn

    Implementation:
    - Grabs location of AirsimCameraFrame
    - Grabs location of of AirsimMovePawn 
    - Moves AirsimCameraFrame to AirsimMovePawn and sets rotation to be the same as AirsimMovePawn
    - Publishes the frame to ROS2 
    """

    def __init__(self) -> None:
        super().__init__('airsim_camera_frame')
        #declare parameters and their types
        self.declare_parameter('airsim_camera_cam_frame', 'camera_frame_2')
        self.declare_parameter('airsim_move_pawn_name', 'AirsimMovePawn_2')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('dt', 0.01)
        
        # this is NED frame
        self.declare_parameter('ned_offset_x', 0.0)
        self.declare_parameter('ned_offset_y', 0.0)
        self.declare_parameter('ned_offset_z', 0.25)
        
        self.cam_frame = self.get_parameter('airsim_camera_cam_frame').get_parameter_value().string_value
        self.unreal_asset = self.get_parameter('airsim_move_pawn_name').get_parameter_value().string_value
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        self.dt = self.get_parameter('dt').get_parameter_value().double_value
        self.ned_offset_x = self.get_parameter('ned_offset_x').get_parameter_value().double_value
        self.ned_offset_y = self.get_parameter('ned_offset_y').get_parameter_value().double_value
        self.ned_offset_z = self.get_parameter('ned_offset_z').get_parameter_value().double_value
        
        
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.init_airsim()
        self.timer = self.create_timer(self.dt, self.update_airsim)
        
    def init_airsim(self) -> None:
        """
        Initialize the Airsim client
        """
        self.client = airsim.VehicleClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.get_logger().info("Connected to AirSim")
        
    def get_location_position(self) -> airsim.Vector3r:
        return self.client.simGetVehiclePose().position
    
    def get_local_orientation(self) -> airsim.Vector3r:
        return self.client.simGetVehiclePose().orientation
    
    def move_camera(self, pose:airsim.Pose) -> None:
        """
        Move the camera to the location and orientation
        """
        self.client.simSetObjectPose(self.unreal_asset, pose)

    def update_airsim(self) -> None:
        """
        We want to look up the transformation of the 
        """
        print("trying to find")
        try:
            transformation = self.tf.lookup_transform(
                self.cam_frame, self.world_frame, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().error(f"Failed to lookup transform: {e}")
            return
                
    
        #remember tf returns in ENU need to convert to NED
        translation = transformation.transform.translation
        enu_rotation = transformation.transform.rotation
        enu_rotation = [enu_rotation.x, enu_rotation.y, 
                        enu_rotation.z, enu_rotation.w]
        print(f"Translation: {translation}")   

        aircraft_position = self.get_location_position()

        # go from ENU to 
        pose = airsim.Pose()
        # pose.position.x_val = translation.y
        # pose.position.y_val = translation.x
        # pose.position.z_val = -translation.z
        
        pose.position.x_val = aircraft_position.x_val + self.ned_offset_x
        pose.position.y_val = aircraft_position.y_val + self.ned_offset_y
        pose.position.z_val = aircraft_position.z_val + self.ned_offset_z
            
        #rotate 
        # rotate_transform = [0, 0, 0, 0]
        # ned_quaternion = tf_transformations.quaternion_multiply(rotate_transform, enu_rotation)
        # pose.orientation.x_val = ned_quaternion[0]
        # pose.orientation.y_val = ned_quaternion[1]
        # pose.orientation.z_val = ned_quaternion[2]
        # pose.orientation.w_val = ned_quaternion[3]
        
        pose.orientation.x_val = enu_rotation[0]
        pose.orientation.y_val = enu_rotation[1]
        pose.orientation.z_val = enu_rotation[2]
        pose.orientation.w_val = enu_rotation[3]
        
        self.move_camera(pose)

def main() -> None:
    rclpy.init()
    node = AirsimCameraFrame()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
        
if __name__ == '__main__':
    main()