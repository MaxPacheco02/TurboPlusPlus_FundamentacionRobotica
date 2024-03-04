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

class SignalGenerator(Node):
    def __init__(self):
        super().__init__('signal_generator')
        # Publishers
        self.signal_publisher_ = self.create_publisher(Float32, 'signal', 10)
        self.time_publisher_ = self.create_publisher(Float32, 'time', 10)
        self.params_publisher_ = self.create_publisher(WaveProperties, 'signal_params', 10)
        
        self.declare_parameter('des_signal', 0)

        timer_period = timer_T  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.params_timer = self.create_timer(0.01, self.params_publisher_timer_callback)
        
        self.sig_type = 1
        self.sig_amplitude = 0.5
        self.sig_frequency = 2.0 # Hz
        self.sig_offset = 0

        self.phase_shift = 0
        self.i = 0.0

        self.config_file = os.path.join(
            get_package_share_directory('courseworks_week2'),
            'config',
            'config_file.yaml'
        )

        with open(self.config_file, 'r') as file:
            self.config_params = yaml.safe_load(file)

    def timer_callback(self):
        time_msg = Float32()
        time_msg.data = self.i
        self.time_publisher_.publish(time_msg)

        self.des_signal = self.get_parameter('des_signal').get_parameter_value().integer_value
        if(self.des_signal > 0 and self.des_signal < 6):
            sig = 'signal' + str(self.des_signal)
            self.sig_type = self.config_params[sig]['type']
            self.sig_amplitude = self.config_params[sig]['amplitude']
            self.sig_frequency = self.config_params[sig]['frequency']
            self.sig_offset = self.config_params[sig]['offset']
        else:
            self.get_logger().error("Incorrect Signal Configuration! Available options go from 1 to 5")

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

        self.signal_publisher_.publish(signal_msg)

        # self.get_logger().info('Time: "%f", sin(t)= "%f"' % (self.i, signal_msg.data))
        self.i += timer_T
    
    def params_publisher_timer_callback(self):
        pass
        wave_properties_msg = WaveProperties()
        waves = ["sine", "square", "sawtooth"]
        wave_properties_msg.type = waves[self.sig_type - 1]
        wave_properties_msg.amplitude = (float)(self.sig_amplitude)
        wave_properties_msg.frequency = (float)(self.sig_frequency)
        wave_properties_msg.offset = (float)(self.sig_offset)
        wave_properties_msg.time = (float)(self.i)
        self.params_publisher_.publish(wave_properties_msg)

def main(args=None):
    rclpy.init(args=args)
    signal_generator = SignalGenerator()
    rclpy.spin(signal_generator)
    signal_generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()