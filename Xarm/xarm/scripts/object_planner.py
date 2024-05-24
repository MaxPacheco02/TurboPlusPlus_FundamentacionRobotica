#!/usr/bin/env python3
import math 

import rclpy
from rclpy.node import Node

from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Vector3, Pose, PoseStamped, TransformStamped, Point
from std_msgs.msg import ColorRGBA, Int8
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

FIG_SCALE = 0.05

X_OFFSET = 0.3
Y_OFFSET = 0.3
Z_OFFSET = 0.0

Z_ADDER = 0 # For the sphere huh

scale = 0.0027
rangeX = 0.5
rangeY = 0.5
upZ = 0.5
bottomZ = 0.0

class ObjectPlannerNode(Node):

    def __init__(self):
        super().__init__('object_planner')

        self.obj_sub_ = self.create_subscription(Float32MultiArray,'obj_pos',self.sub_callback, 10)
        self.obj_sub_
        self.obj_xarm_marker_pub_ = self.create_publisher(Marker, '/xarm_obj_marker', 10)
        self.obj_xarm_pose_pub_ = self.create_publisher(PoseStamped, '/xarm_obj_pose', 10)
       


    def sub_callback(self, msg):
        y = (msg.data[0] + msg.data[3])/2 * scale + Y_OFFSET # y -> X
        z = -1*(msg.data[1] + msg.data[4])/2 * scale + Z_OFFSET # z -> Y 
        x = (msg.data[2] + msg.data[5])/2 * scale + X_OFFSET # x -> Z

        x = rangeX if x >= rangeX else x
        x = -rangeX if x <= -rangeX else x
        y = rangeY if y >= rangeY else y
        y = -rangeY if y <= -rangeY else y
        z = upZ if z >= upZ else z
        z = bottomZ if z <= bottomZ else z


        # Pose
        poseObj = PoseStamped() 
        poseObj.header.frame_id = "world"
        poseObj.pose.position.x = x
        poseObj.pose.position.y = y
        poseObj.pose.position.z = z
        poseObj.pose.orientation.w = 0.0
        poseObj.pose.orientation.x = 1.0
        poseObj.pose.orientation.y = 0.0
        poseObj.pose.orientation.z = 0.0

        #Marker
        objMarker = Marker()

        objMarker.color = ColorRGBA(r = 1.0, g = 0.0, b = 0.0, a = 1.0)
        objMarker     
        objMarker.header.frame_id = 'world'
        objMarker.id = 0
        objMarker.type = 2
        objMarker.action = 0
        objMarker.scale = Vector3(x = 0.15, y = 0.15, z = 0.15)
        objMarker.pose.position = Point(x = x, y = y, z = z)

        self.obj_xarm_marker_pub_.publish(objMarker) 
        self.obj_xarm_pose_pub_.publish(poseObj) 

        return

def main(args=None):
    rclpy.init(args=args)

    object_planner = ObjectPlannerNode()
    rclpy.spin(object_planner)
    object_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
