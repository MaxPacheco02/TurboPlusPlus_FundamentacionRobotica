

#ros2 run Final_challenge final_challenge --ros-args --params-file ~/ros_ws/src/Final_challenge/config/params.yaml
#ros2 param set /final_challenge sine_wave.frequency 1.0


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np
from scipy import signal as sig


class Final_Challenge(Node):
    def __init__(self):
        super().__init__('final_challenge') 

        #Node initalization message
        self.get_logger().info('final_challenge node successfully initialized!!')

        #Parameters
        self.declare_parameters(
            namespace='', 
            parameters=[ 
                ('wave_type', rclpy.Parameter.Type.INTEGER),
                ('wave_time', rclpy.Parameter.Type.DOUBLE),
                ('sine_wave.frequency', rclpy.Parameter.Type.DOUBLE),
                ('sine_wave.offset', rclpy.Parameter.Type.DOUBLE),
                ('sine_wave.amplitude', rclpy.Parameter.Type.DOUBLE),
                ('square_wave.frequency', rclpy.Parameter.Type.DOUBLE),
                ('square_wave.offset', rclpy.Parameter.Type.DOUBLE),
                ('square_wave.amplitude', rclpy.Parameter.Type.DOUBLE),
                ('saw_wave.frequency', rclpy.Parameter.Type.DOUBLE),
                ('saw_wave.offset', rclpy.Parameter.Type.DOUBLE),
                ('saw_wave.amplitude', rclpy.Parameter.Type.DOUBLE),
                ('unit.amplitude', rclpy.Parameter.Type.DOUBLE)
        ])

        #Topics
        #-----------------------------------------------------------------------------
        #Publisher for duty cycle setpoint
        self.signal_publisher = self.create_publisher(Float32, 'setpoint', 10)

        #Angular speed subscriber
        self.signal_subscriber = self.create_subscription(Float32, 'angular_speed', self.signal_callback, 10)
        #-----------------------------------------------------------------------------

        #Timer for duty cycle setpoint topic callback function
        self.timer_period_1 = 0.1
        self.timer1 = self.create_timer(self.timer_period_1, self.timer_callback_P)

        # VARIABLES
        self.signal = Float32()
        self.time = 0.0
        self.type = 0
        self.amplitude = 0.0
        self.frequency = 0.0
        self.offset = 0.0

    def timer_callback_P(self):
        #Check the type of signal to generate and publish (Default is sine wave)
        self.type = self.get_parameter('wave_type').get_parameter_value().integer_value
        if(self.type == 1):
            self.frequency = self.get_parameter('sine_wave.frequency').get_parameter_value().double_value
            self.offset = self.get_parameter('sine_wave.offset').get_parameter_value().double_value
            self.amplitude = self.get_parameter('sine_wave.amplitude').get_parameter_value().double_value
            self.signal.data = (self.amplitude * np.sin(self.frequency*self.time)) + self.offset
        elif(self.type == 2):
            self.frequency = self.get_parameter('square_wave.frequency').get_parameter_value().double_value
            self.offset = self.get_parameter('square_wave.offset').get_parameter_value().double_value
            self.amplitude = self.get_parameter('square_wave.amplitude').get_parameter_value().double_value
            self.signal.data = (self.amplitude * sig.square(self.frequency*self.time)) + self.offset
        elif(self.type == 3):
            self.frequency = self.get_parameter('saw_wave.frequency').get_parameter_value().double_value
            self.offset = self.get_parameter('saw_wave.offset').get_parameter_value().double_value
            self.amplitude = self.get_parameter('saw_wave.amplitude').get_parameter_value().double_value
            self.time += 1/self.frequency
            self.signal.data = (self.amplitude * sig.sawtooth(self.frequency * self.time/10)) + self.offset
        else:
            self.amplitude = self.get_parameter('unit.amplitude').get_parameter_value().double_value
            self.signal.data = self.amplitude

        self.time = self.time + self.timer_period_1
        self.signal_publisher.publish(self.signal)

    def signal_callback(sel, angular_speed: Float32):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = Final_Challenge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
