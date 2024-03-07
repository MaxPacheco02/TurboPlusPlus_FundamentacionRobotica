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
from geometry_msgs.msg import Vector3, Pose, PoseStamped, TransformStamped
from std_msgs.msg import ColorRGBA, Int8
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

import speech_recognition as sr

FIG_SCALE = 0.05

X_OFFSET = 0.2
Y_OFFSET = 0.2
Z_OFFSET = 0.2

Z_ADDER = 0 # For the sphere huh

class TrajectoryPlannerNode(Node):
    published = True
    shape_option = 0
    drawing = MarkerArray()
    i = 0

    def __init__(self):
        global Z_ADDER, FIG_SCALE
        super().__init__('trajectory_planner')

        recognized = False
        while not recognized:
            r= sr.Recognizer()
            with sr.Microphone() as source:
                    print('Choose a figure:')
                    print('1: Cone')
                    print('2: Cube')
                    print('3: Cylinder')
                    print('4: Hexagonal Prism')
                    print('5: Prism')
                    print('6: Sphere')
                    print('7: Squared Pyramid')
                    print('8: Triangular Prism')
                    print('9: Triangular Pyramid')
                    r.pause_threshold = 1
                    r.adjust_for_ambient_noise(source)
                    audio = r.listen(source)
            try:
                        phrase = (r.recognize_google(audio, language="es-419").lower())
                        print('You said: ' + phrase + '\n')
                        opt = ''
                        if(phrase == 'uno' or phrase == '1'):
                            opt = 'cone'
                        if(phrase == 'dos' or phrase == '2'):
                            opt = 'cube'
                        if(phrase == 'tres' or phrase == '3'):
                            opt = 'cylinder'
                        if(phrase == 'cuatro' or phrase == '4'):
                            opt = 'hexagonal_prism'
                        if(phrase == 'cinco' or phrase == '5'):
                            opt = 'prism'
                            FIG_SCALE = 0.1
                        if(phrase == 'seis' or phrase == '6'):
                            opt = 'sphere'
                            Z_ADDER = 1
                        if(phrase == 'siete' or phrase == '7'):
                            opt = 'squared_pyramid'
                        if(phrase == 'ocho' or phrase == '8'):
                            opt = 'triangular_prism'
                        if(phrase == 'nueve' or phrase == '9'):
                            opt = 'triangular_pyramid'
                        
                        if opt != '':
                            recognized = True
                        else:
                             print('Try again!')

            except sr.UnknownValueError:
                    print('Your last command couldn\'t be heard')




        
        self.path_pub_ = self.create_publisher(Path, '/xarm_planned_path', 10)
        self.marker_pub_ = self.create_publisher(MarkerArray, '/mk_arr', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.shape_option_sub = self.create_subscription(
            Int8,
            "/shape_option",
            self.shape_option_callback,
            10)
        
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.wp_file_ = os.path.join(
            get_package_share_directory('xarm'),
            'config',
            opt + '.csv'
            # 'prueba.csv'
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
                # pose_stmpd.pose.position.x = float(row[0])
                # pose_stmpd.pose.position.y = float(row[1])
                # pose_stmpd.pose.position.z = float(row[2])
                pose_stmpd.pose.orientation.w = 0.0
                pose_stmpd.pose.orientation.x = 1.0
                pose_stmpd.pose.orientation.y = 0.0
                pose_stmpd.pose.orientation.z = 0.0

                self.path_.poses.append(pose_stmpd)
        pose_stmpd = PoseStamped()
        pose_stmpd.header.frame_id = "world"
        pose_stmpd.pose.position.x = X_OFFSET
        pose_stmpd.pose.position.y = Y_OFFSET
        pose_stmpd.pose.position.z = 0.15
        pose_stmpd.pose.orientation.w = 0.0
        pose_stmpd.pose.orientation.x = 1.0
        pose_stmpd.pose.orientation.y = 0.0
        pose_stmpd.pose.orientation.z = 0.0

        self.path_.poses.append(pose_stmpd)


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
                dot.color = ColorRGBA()
                dot.color.r = 1.0
                dot.color.g = 0.0
                dot.color.b = 0.0
                dot.color.a = 1.0
                
                dot.header.frame_id = 'world'
                dot.id = self.i
                self.i+=1
                dot.type = 2
                dot.action = 0
                dot.scale = Vector3()
                dot.scale.x = 0.01
                dot.scale.y = 0.01
                dot.scale.z = 0.01
                dot.pose.position.x = -t.transform.translation.x
                dot.pose.position.y = t.transform.translation.y
                dot.pose.position.z = t.transform.translation.z - 0.01
                self.drawing.markers.append(dot)
                self.marker_pub_.publish(self.drawing)

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return


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