#!/usr/bin/env python3
import rclpy
import numpy as np

from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String

ODOMETRY_GLOBAL_TOPIC = "/odometry/global"
ODOMETRY_LOCAL_TOPIC = "/odometry/local"
ALERTS_TOPIC = "/alerts"

class DetectorNode(Node):
    def __init__(self):
        super().__init__("spoofing_jamming_detector_node")
        # Subscribers
        _ = self.create_subscription(Odometry, ODOMETRY_GLOBAL_TOPIC, self.odom_global_callback, 10)
        _ = self.create_subscription(Odometry, ODOMETRY_LOCAL_TOPIC, self.odom_local_callback, 10)
        
        # Publishers
        self.warn_pub = self.create_publisher(String, ALERTS_TOPIC, 10)
        
        self.global_pos = None
        self.local_pos = None
        self.offset = None
        self.threshold = cm2m(50.0)
        # Config numpy prints
        np.set_printoptions(precision=6, suppress=True)

    def odom_global_callback(self, odom: Odometry):
        new_pose_received = from_odom_to_np(odom)
        
        if self.global_pos is None:
            print("Storing initial GLOBAL Pose!", new_pose_received)
            self.global_pos = new_pose_received
            return
        
        if self.jump_detection(new_pose_received, _global=True):
            self.publish_warning(_global=True, new_pose=new_pose_received)
        
        self.global_pos = new_pose_received

    def odom_local_callback(self, odom: Odometry):
        new_pose_received = from_odom_to_np(odom)
        
        if self.local_pos is None:
            print("Storing initial LOCAL Pose!", new_pose_received.flatten())
            self.local_pos = new_pose_received
            return
        
        if self.jump_detection(new_pose_received, _global=False):
            self.publish_warning(_global=False, new_pose=new_pose_received)
        
        self.local_pos = new_pose_received        

    def jump_detection(self, new_pose: np.ndarray, _global: bool) -> bool:
        self.offset = np.abs(self.global_pos - new_pose) if _global else np.abs(self.local_pos - new_pose)
        # Check if each element in the result is above threshold
        return np.any(self.offset >= self.threshold)

    def publish_warning(self, _global, new_pose) -> None:
        if _global:
            message = (
                "Jump in GLOBAL position!"
                f"Offset: {self.offset.flatten()}"
                f"Prev. pose: {self.global_pos.flatten()}"
                f"New pose: {new_pose.flatten()}"
            )
            self.get_logger().warn("Jump detected in GLOBAL position!")
        else:
            message = (
                "Jump in LOCAL position!"
                f"Offset: {self.offset.flatten()}"
                f"Prev. pose: {self.local_pos.flatten()}"
                f"New pose: {new_pose.flatten()}"
            )
            self.get_logger().info("Jump detected in LOCAL position!")
        
        msg = String()
        msg.data = message
        self.warn_pub.publish(msg)

def from_odom_to_np(msg) -> np.ndarray:
    """Convert position of an Odometry message in numpy array"""
    return np.array([msg._pose._pose._position._x, msg._pose._pose._position._y, msg._pose._pose._position._z]).reshape(3,1) 

def cm2m(x: float) -> float:
    """Convert centimeters to meters."""
    return x / 100.0

def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()