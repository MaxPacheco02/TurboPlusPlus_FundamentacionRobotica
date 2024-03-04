#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Int8

import os
from ament_index_python.packages import get_package_share_directory

import math
import yaml
from scipy import signal

from courseworks_interfaces.msg import WaveProperties

timer_T = 0.001 # 1 KHz -> 1/1000 s

class Reconstruction(Node):
    def __init__(self):
        super().__init__('reconstruction')
        # Publishers
        self.signal_reconstructed_publisher_ = self.create_publisher(Float32, 'signal_reconstructed', 10)

        # Subscribers
        self.signal_params_subscriber_ = self.create_subscription(
            WaveProperties,
            'signal_params',
            self.signal_params_callback,
            10)

        timer_period = timer_T  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.sig_type = 1
        self.sig_amplitude = 0.5
        self.sig_frequency = 2.0 # Hz
        self.sig_offset = 0

        self.phase_shift = 0
        self.i = 0.0

    def timer_callback(self):
        signal_msg = Float32()
        if self.sig_type == 1:
            # y = sin(bx)
            # The period = 2pi/b -> Frequency = b / (2pi)
            # Initial desired frequency is 2Hz -> b = 2pi * frequency = 4pi
            signal_msg.data = math.sin(self.i*2*math.pi*self.sig_frequency + self.phase_shift) * self.sig_amplitude + self.sig_offset
        elif self.sig_type == 2:
            signal_msg.data = signal.square(self.i*2*math.pi*self.sig_frequency + self.phase_shift) * self.sig_amplitude + self.sig_offset
        elif self.sig_type == 3:
            signal_msg.data = signal.sawtooth(self.i*2*math.pi*self.sig_frequency + self.phase_shift) * self.sig_amplitude + self.sig_offset
        self.signal_reconstructed_publisher_.publish(signal_msg)

        # self.get_logger().info('Time: "%f", sin(t)= "%f"' % (self.i, signal_msg.data))
        # self.i += timer_T

    def signal_params_callback(self, msg):
        waves = ["sine", "square", "sawtooth"]
        self.sig_type = waves.index(msg.type)+1
        self.sig_amplitude = msg.amplitude
        self.sig_frequency = msg.frequency
        self.sig_offset = msg.offset 
        self.i = msg.time 
        print(self.sig_type, self.sig_amplitude, self.sig_frequency, self.sig_offset, self.i)

def main(args=None):
    rclpy.init(args=args)
    reconstruction = Reconstruction()
    rclpy.spin(reconstruction)
    reconstruction.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()