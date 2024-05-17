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

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from ament_index_python.packages import get_package_share_directory

from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Vector3, Pose, PoseStamped, TransformStamped, Point
from std_msgs.msg import ColorRGBA, Int8
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

FIG_SCALE = 0.05

X_OFFSET = 0.3
Y_OFFSET = 0.0
Z_OFFSET = 0.4

Z_ADDER = 0 # For the sphere huh

class TrajectoryPlannerNode(Node):
    shape_option = 0
    drawing = MarkerArray()
    i = 0

    def __init__(self):
        global Z_ADDER, FIG_SCALE
        super().__init__('trajectory_planner')

        self.path_pub_ = self.create_publisher(Path, '/xarm_planned_path', 10)
        self.marker_pub_ = self.create_publisher(MarkerArray, '/mk_arr', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        timer_period = 0.01
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
                pose_stmpd.pose.position.x = float(row[0])*FIG_SCALE + X_OFFSET
                pose_stmpd.pose.position.y = float(row[1])*FIG_SCALE + Y_OFFSET
                pose_stmpd.pose.position.z = (float(row[2]) + Z_ADDER)*FIG_SCALE + Z_OFFSET
                pose_stmpd.pose.orientation.w = 0.0
                pose_stmpd.pose.orientation.x = 1.0
                pose_stmpd.pose.orientation.y = 0.0
                pose_stmpd.pose.orientation.z = 0.0

                self.path_.poses.append(pose_stmpd)

        self.path_pub_.publish(self.path_)

    def timer_callback(self):
        from_frame_rel = 'world'
        to_frame_rel = 'link_eef'

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            print(f'Pose: x = {t.transform.translation.x}, y = {t.transform.translation.y}, z = {t.transform.translation.z}')
            
            if t.transform.translation.z >= Z_OFFSET - 0.001:
                dot = Marker()
                dot.color = ColorRGBA(r = 1.0, g = 0.0, b = 0.0, a = 1.0)
                
                dot.header.frame_id = 'world'
                dot.id = self.i
                self.i+=1
                dot.type = 2
                dot.action = 0
                dot.scale = Vector3(x = 0.01, y = 0.01, z = 0.01)
                dot.pose.position = Point(x = -t.transform.translation.x, y = t.transform.translation.y, z = t.transform.translation.z - 0.01)

                self.drawing.markers.append(dot)
                self.marker_pub_.publish(self.drawing)

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

def main(args=None):
    rclpy.init(args=args)

    trajectory_planner = TrajectoryPlannerNode()
    rclpy.spin(trajectory_planner)
    trajectory_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()