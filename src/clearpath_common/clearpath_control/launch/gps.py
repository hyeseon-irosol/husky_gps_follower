#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 20 11:52:01 2024

@author: hyeseonl
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from geopy.distance import geodesic
import tf_transformations
import math

class GPSNavigator(Node):
    def __init__(self):
        super().__init__('gps_navigator')
        self.target_latitude = -22.986676   # Example target latitude
        self.target_longitude = -43.202501 # Example target longitude
        self.target_altitude = 0.0        # Optional: altitude, set 0 if not used

        self.current_latitude = None
        self.current_longitude = None

        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            '/gps/fix',  # Change this to the correct GPS topic name
            self.gps_callback,
            10
        )

        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',  # Topic for sending goal positions to the Nav2 stack
            10
        )

    def gps_callback(self, msg):
        self.current_latitude = msg.latitude
        self.current_longitude = msg.longitude

        if self.current_latitude and self.current_longitude:
            self.move_to_target()

    def move_to_target(self):
        # Calculate distance to target
        current_location = (self.current_latitude, self.current_longitude)
        target_location = (self.target_latitude, self.target_longitude)

        distance = geodesic(current_location, target_location).meters

        if distance < 1.0:
            self.get_logger().info("Reached the target location")
            return

        # Convert GPS to local PoseStamped target (assuming flat Earth and no orientation)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        # Example conversion (not accurate in real scenarios, use a better conversion method like UTM)
        goal_pose.pose.position.x = distance * (self.target_latitude - self.current_latitude)
        goal_pose.pose.position.y = distance * (self.target_longitude - self.current_longitude)
        goal_pose.pose.position.z = 0.0

        # Assuming no rotation for simplicity, adjust as needed
        yaw = math.atan2(goal_pose.pose.position.y, goal_pose.pose.position.x)
        quaternion = tf_transformations.quaternion_from_euler(0, 0, yaw)
        goal_pose.pose.orientation.x = quaternion[0]
        goal_pose.pose.orientation.y = quaternion[1]
        goal_pose.pose.orientation.z = quaternion[2]
        goal_pose.pose.orientation.w = quaternion[3]

        # Publish the goal pose
        self.pose_publisher.publish(goal_pose)
        self.get_logger().info(f"Moving to target: x={goal_pose.pose.position.x}, y={goal_pose.pose.position.y}")

def main(args=None):
    rclpy.init(args=args)
    gps_navigator = GPSNavigator()
    rclpy.spin(gps_navigator)
    gps_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()