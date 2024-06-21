#!/usr/bin/env python3

import math  
import numpy as np

def get_euler_from_quaternion(x:float, y:float, z:float, w:float) -> tuple:
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

 
def get_quaternion_from_euler(roll:float, pitch:float, yaw:float) -> list:
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """

  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

  return [qx, qy, qz, qw]


#rotation 3d
def rot3d(roll, pitch, yaw):
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])

    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])

    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])

    R = np.dot(R_x, np.dot(R_y, R_z))
    return R

def rot2d(psi):
    return np.array([[np.cos(psi), -np.sin(psi)],
                     [np.sin(psi), np.cos(psi)]])


def convertENUToNED(x_enu:float, y_enu:float, z_enu:float) -> list:
    """converts from ENU to NED"""
    ned =  np.zeros(3, dtype=np.float64)
    ned[0] = y_enu
    ned[1] = x_enu
    ned[2] = -z_enu
    return ned

def convertNEDToENU(x_ned:float, y_ned:float, z_ned:float) -> list:
    """converts from NED to ENU"""
    #create 3,1 array
    enu = np.zeros(3, dtype=np.float64)
    enu[0] = y_ned
    enu[1] = x_ned
    enu[2] = -z_ned
    return enu

    

"""
Broadcast static effector reference frame wrt to base frame of UAV
"""

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

class EffectorFrameScaled(Node):
    
        def __init__(self):
            super().__init__('effector_frame_scaled')

            self.br = TransformBroadcaster(self)    
            self.declare_parameter('x', 0.0)
            self.declare_parameter('y', 0.0)
            self.declare_parameter('z', 0.0)

            self.declare_parameter('roll', 0.0)
            self.declare_parameter('pitch', 0.0)
            self.declare_parameter('yaw', 0.0)
            self.declare_parameter('parent_frame', 'aircraft_frame_scaled') 
            self.declare_parameter('child_frame', 'effector_frame_scaled')
            self.declare_parameter('rate', 30.0)

            self.x = self.get_parameter('x').value
            self.y = self.get_parameter('y').value
            self.z = self.get_parameter('z').value

            self.roll = self.get_parameter('roll').value
            self.pitch = self.get_parameter('pitch').value
            self.yaw = self.get_parameter('yaw').value

            self.quaternion = get_quaternion_from_euler(
                float(self.roll), float(self.pitch), float(self.yaw))

            self.parent_frame = self.get_parameter('parent_frame').value
            self.child_frame = self.get_parameter('child_frame').value

            self.rate = self.get_parameter('rate').value
            self.timer = self.create_timer(1/self.rate, self.broadcastTransform)

        def broadcastTransform(self):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.parent_frame
            t.child_frame_id = self.child_frame
            t.transform.translation.x = self.x/SCALE_SIM
            t.transform.translation.y = self.y/SCALE_SIM
            t.transform.translation.z = self.z/SCALE_SIM
            t.transform.rotation.x = self.quaternion[0]
            t.transform.rotation.y = self.quaternion[1]
            t.transform.rotation.z = self.quaternion[2]
            t.transform.rotation.w = self.quaternion[3]
    
            self.br.sendTransform(t)    


def main(args=None):
    rclpy.init(args=args)
    node = EffectorFrameScaled()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
