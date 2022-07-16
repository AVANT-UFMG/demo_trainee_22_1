#coding: utf-8

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

import numpy as np
from scipy.spatial.transform import Rotation

class Odom(Node):
    def __init__(self):
        super().__init__('odom')

        # self.declare_parameters(['base', 'wheel_radius'])

        # self.base = self.get_parameter('base').get_parameter_value().double_value
        # self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value

        self.base = 0.4150
        self.wheel_radius = 0.0975
        
        self.wheel_state_sub = self.create_subscription(JointState, 'joint_state', callback=self.wheel_state_cb)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        self.pose = np.array([0, 0, 0])
        self.last_stamp = 0

    def wheel_state_cb(self, msg: JointState):
        wheel_state = dict(zip(msg.name, msg.velocity))
        omega_r = wheel_state['right']
        omega_l = wheel_state['left']

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_stamp != 0:
            dt = stamp - self.last_stamp
            self.update_pose(omega_r, omega_l, dt)

        self.last_stamp = stamp

    def update_pose(self, omega_r: float, omega_l: float, dt: float):
        # usando variáveis auxiliares para facilitar a leitura
        r = self.wheel_radius   # raio da roda
        l = self.base / 2       # metade da largura do robô
        theta = self.pose[2]    # orientação atual

        # modelo cinemático de um robô diferencial
        kin_model = np.array([  [r * np.cos(theta)/2, r * np.cos(theta)/2],
                                [r * np.sin(theta)/2, r * np.sin(theta)/2],
                                [r/l,                -r/l                ]])
        
        omega = np.array([omega_r, omega_l])

        self.pose_dot = np.matmul(kin_model, omega) # pose_dot é a derivada do pose (velocidades)

        # atualiza o pose multiplicando a velocidade pela variação no tempo
        self.pose += self.pose_dot * dt

    def publish_odom(self):
        msg = Odometry()
        msg.pose.pose.position.x = self.pose[0]
        msg.pose.pose.position.y = self.pose[1]
        
        quat = Rotation.from_euler('xyz', [0, 0, self.pose[2]], degrees=False)
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

        self.odom_pub.publish(msg)
        

def main():
    rclpy.init(args=None)
    odom = Odom()
    rclpy.spin()

    odom.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()