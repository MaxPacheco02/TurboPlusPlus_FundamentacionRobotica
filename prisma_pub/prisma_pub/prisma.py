import rclpy
from rclpy.node import Node
from prisma_msgs.msg import Prism

import numpy as np

class Route(Node):

    def __init__(self):
        super().__init__('prisma') 

        # NODE INITIALIZATION MSG
        self.get_logger().info('signal_generator node successfully initialized!!')

        # TOPIC
        #-----------------------------------------------------------------------------
        # Publisher for prisma position and orientation
        self.prisma_publisher = self.create_publisher(Prism, 'prisma_cords', 10)

        # timer for publish callback
        self.timer_period_1 = 0.1
        self.timer1 = self.create_timer(self.timer_period_1, self.timer_callback_M)
        #-----------------------------------------------------------------------------

        # SETUP
        # ------------------------------------------------------
        # box dimensions
        Lengthb = 5
        Heightb = 5
        Widthb = 5

        la = Lengthb/2
        ha = Heightb/2
        wa = Widthb/2

        # frame vectors
        PCMa = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        P1a = PCMa
        P2a = PCMa
        P3a = PCMa
        self.ORIENI = [PCMa, P1a, P2a, P3a]
        # ------------------------------------------------------

        # INITIAL CONFIGURATION 
        # ------------------------------------------------------
        # position of cm and contact points in space
        PCM = np.array([0, 0, 0])
        P1 = np.array([0, -la/2, ha])
        P2 = np.array([0, la/2, ha])
        P3 = np.array([0, 0, -ha])
        self.POSI =  [PCM, P1, P2, P3]
        # ------------------------------------------------------ 

        # FINAL CONFIGURATION
        # ------------------------------------------------------
        # translation vector
        self.dis = [5, 0, 0]
        # radian rotation
        self.rot = [0, -np.pi/4, 0]
        # ------------------------------------------------------

        self.i = 0
        self.frames = 100

        self.prisma_info = Prism()


    def timer_callback_M(self):
        div = (self.i/self.frames)
        xang = self.rot[0] * div 
        yang = self.rot[1] * div 
        zang = self.rot[2] * div 
        xdis = self.dis[0] * div
        ydis = self.dis[1] * div
        zdis = self.dis[2] * div
        
        disN = np.array([xdis, ydis, zdis])
        
        # ACTION
        # ------------------------------------------------------
        # ROTATION MATRICES
        Rx = np.array([[1, 0, 0], [0, np.cos(xang), -np.sin(xang)], [0, np.sin(xang), np.cos(xang)]])
        Ry = np.array([[np.cos(yang), 0, np.sin(yang)], [0, 1, 0], [-np.sin(yang), 0, np.cos(yang)]])
        Rz = np.array([[np.cos(zang), -np.sin(zang), 0], [np.sin(zang), np.cos(zang), 0], [0, 0, 1]])
        R = Rx @ Ry @ Rz
        
        # PERFORM ROTATION
        PCMar = R @ np.array(self.ORIENI[0])
        P1ar = R @ np.array(self.ORIENI[1])
        P2ar = R @ np.array(self.ORIENI[2])
        P3ar = R @ np.array(self.ORIENI[3])
        
        # PERFORM TRANSLATION
        PCMd = np.array(self.POSI[0]) + disN
        # Rotate and translate the entire position vector
        P1d = np.dot(R, np.array(self.POSI[1])) + disN
        P2d = np.dot(R, np.array(self.POSI[2])) + disN
        P3d = np.dot(R, np.array(self.POSI[3])) + disN

        self.i += 1
        self.prisma_info.center.x = PCMd[0]
        self.prisma_info.center.y = PCMd[1]
        self.prisma_info.center.z = PCMd[2]

        self.prisma_info.p1.x = P1d[0]
        self.prisma_info.p1.y = P1d[1]
        self.prisma_info.p1.z = P1d[2]

        self.prisma_info.p2.x = P2d[0]
        self.prisma_info.p2.y = P2d[1]
        self.prisma_info.p2.z = P2d[2]

        self.prisma_info.p3.x = P3d[0]
        self.prisma_info.p3.y = P3d[1]
        self.prisma_info.p3.z = P3d[2]

        if self.i <= (self.frames + 1):
            self.prisma_publisher.publish(self.prisma_info)

def main(args=None):
    rclpy.init(args=args)
    m_p = Route()
    rclpy.spin(m_p)
    m_p.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
