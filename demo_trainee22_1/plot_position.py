# coding: utf-8

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import numpy as np
from matplotlib import pyplot as plt

class Plotter(Node):
    def __init__(self):
        super().__init__('plot_odom')

        self.data_gt = list()       # 'ground truth' (valor exato dado pelo simulador)
        self.data_raw = list()      # resultado da odometria sozinha
        self.data_filtered = list() # resultado com o filtro

        self.WORLD = 30

        self.fig = plt.figure()
        self.ax_gt = self.fig.add_subplot(311, aspect='equal')
        self.ax_raw = self.fig.add_subplot(312, aspect='equal')
        self.ax_filtered = self.fig.add_subplot(313, aspect='equal')

        self.gt_odom_sub = self.create_subscription(Odometry, 'gt_odom', self.gt_odom_cb)
        self.raw_odom_sub = self.create_subscription(Odometry, 'odom', self.raw_odom_cb)
        self.filtered_odom_sub = self.create_subscription(Odometry, 'odom/filtered', self.filtered_odom_cb)

        # self.timer = self.create_timer(0.1, callback=self.plot_data)

    def gt_odom_cb(self, msg: Odometry):
        pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.data_gt.append(pos)

    def raw_odom_cb(self, msg: Odometry):
        pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.data_raw.append(pos)

    def filtered_odom_cb(self, msg: Odometry):
        pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.data_filtered.append(pos)
    
    def plot_data(self):
        self.__plot(self.ax_gt, self.data_gt)
        self.__plot(self.ax_raw, self.data_raw)
        self.__plot(self.ax_filtered, self.data_filtered)

        plt.axis((-self.WORLD, self.WORLD, -self.WORLD, self.WORLD))
        plt.legend()
        plt.show()

    def __plot(self, ax, title: str, label: str, data: list):
        data_arr = np.transpose(np.array(data))
        ax.plot(data_arr[0], data_arr[1], 'r', label=label)
        ax.set_title(title)
    
def main():
    rclpy.init(args=None)
    node = Plotter()
    rclpy.spin()

    node.plot_data()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
