#!/usr/bin/env python3
''' ----------------------------------------------------------------------------
 * @file: trajectory_planner.py
 * @date: March 4, 2023
 * @author: Max Pacheco
 *
 * -----------------------------------------------------------------------------
'''

import os
import csv
import math 

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import Vector3, Pose, PoseStamped
from std_msgs.msg import ColorRGBA, Int8
from visualization_msgs.msg import Marker, MarkerArray
from control_msgs.msg import JointTrajectoryControllerState

class TrajectoryPlannerNode(Node):
    published = True
    shape_option = 0

    def __init__(self):
        super().__init__('trajectory_planner')
        self.path_pub_ = self.create_publisher(Path, '/xarm_planned_path', 10)
        self.marker_pub_ = self.create_publisher(MarkerArray, '/mk_arr', 10)

        self.shape_option_sub = self.create_subscription(
            Int8,
            "/shape_option", 
            self.shape_option_callback,
            10)
        
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.wp_file_ = os.path.join(
            get_package_share_directory('xarm'),
            'config',
            'cone.csv'
        )

        self.path_ = Path()
        self.path_.header.frame_id = "world"
        self.path_.header.stamp = self.get_clock().now().to_msg()
        with open(self.wp_file_, 'r') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')

            for row in csv_reader:
                pose_stmpd = PoseStamped()
                pose_stmpd.header.frame_id = "world"
                pose_stmpd.pose.position.x = float(row[0])*0.1 + 0.2
                pose_stmpd.pose.position.y = float(row[1])*0.1 + 0.2
                pose_stmpd.pose.position.z = float(row[2])*0.1 + 0.2
                pose_stmpd.pose.orientation.w = 0.0
                pose_stmpd.pose.orientation.x = 1.0
                pose_stmpd.pose.orientation.y = 0.0
                pose_stmpd.pose.orientation.z = 0.0

                self.path_.poses.append(pose_stmpd)


    def timer_callback(self):
        if not self.published:
            self.path_pub_.publish(self.path_)
            self.published = True

    def shape_option_callback(self, msg):
        if msg.data != self.shape_option:
            self.published = False
            self.shape_option = msg.data
    
def main(args=None):
    rclpy.init(args=args)

    trajectory_planner = TrajectoryPlannerNode()
    rclpy.spin(trajectory_planner)
    trajectory_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()