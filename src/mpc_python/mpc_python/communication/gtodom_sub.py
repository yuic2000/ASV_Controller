#!/usr/bin/env python3
import numpy as np
from mpc_python.src.utils.LieGroup import quat2rotm, rotm2quat
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist

class GtOdomSubscriberNode(Node):
    def __init__(self):
        super().__init__('ground_truth_odometry_subscriber')
        self.odom_topic = "/wamv/sensors/position/ground_truth_odometry"
        self.get_logger().info(f"Subscribe to odometry topic: {self.odom_topic}")
        
        self.translation_odom_src_to_body = np.array([-532.64, 161.91, -0.07])  # for output.bag
        self.rotation_odom_src_to_body = np.array([1, 0, 0, 0])

        self.odom_src2body = np.block([
            [quat2rotm(self.rotation_odom_src_to_body), self.translation_odom_src_to_body.reshape(-1, 1)],
            [np.zeros((1, 3)), np.ones(1)]
        ])

        self.gtodom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 1000)
        self.body_coord = None
        self.x_body = None
        self.x_odom = None

    def odom_callback(self, msg):
        # position (world frame)
        p_x = msg.pose.pose.position.x
        p_y = msg.pose.pose.position.y
        p_z = msg.pose.pose.position.z
        self.p_odom = np.array([p_x, p_y, p_z])

        # orientation (world frame)
        q_x = msg.pose.pose.orientation.x
        q_y = msg.pose.pose.orientation.y
        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w
        self.q_odom = np.array([q_x, q_y, q_z, q_w])

        # linear velocity (body frame)
        v_x = msg.twist.twist.linear.x
        v_y = msg.twist.twist.linear.y
        v_z = msg.twist.twist.linear.z
        self.v_body = np.array([v_x, v_y, v_z])

        # angular velocity (body frame)
        w_x = msg.twist.twist.angular.x
        w_y = msg.twist.twist.angular.y
        w_z = msg.twist.twist.angular.z
        self.w_body = np.array([w_x, w_y, w_z])

        # self.get_logger().info(f"Seq: [{msg.header.stamp.sec}]")
        # self.get_logger().info(f"Position: {self.p_odom}")
        # self.get_logger().info(f"Orientation: {self.p_odomq}")
        # self.get_logger().info(f"Linear Vel: {self.v_odom}")
        # self.get_logger().info(f"Angular Vel: {self.w_odom}")

        # world frame
        self.odomsrc_coord = np.block([
            [quat2rotm(self.q_odom), self.p_odom.reshape(-1, 1)],
            [np.zeros((1, 3)), np.ones(1)]
        ])

        self.body_coord = np.linalg.inv(self.odom_src2body) @ self.odomsrc_coord

        self.q_body = np.array(rotm2quat(self.body_coord[:3, :3]))
        self.p_body = self.body_coord[:3, 3].flatten()

        self.x_body = np.concatenate([self.q_body, self.p_body, self.w_body, self.v_body], axis=None)
        self.x_odom = np.concatenate([self.q_odom, self.p_odom, self.w_body, self.v_body], axis=None)

    def get_x_body(self):
        return self.x_body

    def get_x_odom(self):
        return self.x_odom


def main(args=None):
    rclpy.init(args=args)
    Odom_sub = GtOdomSubscriberNode()

    def timer_callback():
        if Odom_sub.body_coord is not None:
            print("x_body:", Odom_sub.get_x_body())
            print("x_odom:", Odom_sub.get_x_odom())

    timer_period = 1.0  # seconds
    timer = Odom_sub.create_timer(timer_period, timer_callback)

    rclpy.spin(Odom_sub)
    Odom_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
