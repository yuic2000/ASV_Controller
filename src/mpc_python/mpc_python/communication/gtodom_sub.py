#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from mpc_python.src.utils.LieGroup import quat2rotm, rotm2quat
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped


class GtOdomSubscriberNode(Node):
    def __init__(self):
        super().__init__('ground_truth_odometry_subscriber')
        self.odom_topic = "/wamv/sensors/position/ground_truth_odometry"
        self.pose_topic = "/gt_pose"
        # self.pose_topic = "/wamv/inekf_odom_correction/pose"
        self.twist_topic = "/wamv/inekf_odom_correction/twist"
        self.get_logger().info(f"Subscribe to odometry topic: {self.odom_topic}")
        self.get_logger().info(f"Subscribe to pose topic: {self.pose_topic}")
        self.get_logger().info(f"Subscribe to twist topic: {self.twist_topic}")
        
        # self.translation_odom_src_to_wrld = np.array([-532.64, 162.5, -0.07])  # for output.bag
        self.translation_odom_src_to_wrld = np.array([0, 0, 0])  # for output.bag
        self.rotation_odom_src_to_wrld = np.array([0, 0, 0, 1])     # x: 0, y: 0, z: 0

        self.odom_pos_src2wrld = np.block([
            [quat2rotm(self.rotation_odom_src_to_wrld), self.translation_odom_src_to_wrld.reshape(-1, 1)],
            [np.zeros((1, 3)), np.ones(1)]
        ])

        self.gtodom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 1000)
        self.pose_sub = self.create_subscription(PoseStamped, self.pose_topic, self.pose_callback, 100)
        self.twist_sub = self.create_subscription(TwistStamped, self.twist_topic, self.twist_callback, 100)

        self.x_wrld = None
        self.x_odom = None
        self.odom_list_pose = []
        self.odom_list_twist = []
        self.pose_list = []
        self.twist_list = []
        self.heading = None


    def odom_callback(self, msg):
        # position (map frame)
        p_x = msg.pose.pose.position.x + 532
        p_y = msg.pose.pose.position.y - 162
        p_z = msg.pose.pose.position.z
        self.p_odom = np.array([p_x, p_y, p_z])

        # orientation (map frame)
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

        # map frame
        self.odomsrc_coord = np.block([
            [quat2rotm(self.q_odom), self.p_odom.reshape(-1, 1)],
            [np.zeros((1, 3)), np.ones(1)]
        ])

        self.wrld_coord = np.linalg.inv(self.odom_pos_src2wrld) @ self.odomsrc_coord

        self.q_wrld = np.array(rotm2quat(self.wrld_coord[:3, :3]))
        self.p_wrld = self.wrld_coord[:3, 3].flatten()

        self.x_wrld = np.concatenate([self.q_wrld, self.p_wrld, self.w_body, self.v_body], axis=None)

        self.odom_list_pose.append(np.array([msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9, *self.q_wrld, *self.p_wrld]))
        self.odom_list_twist.append(np.array([msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9, *self.w_body, *self.v_body]))

    def get_x_wrld(self):
        return self.x_wrld
    
    def pose_callback(self, msg):

        # Extract quaternion from PoseStamped message
        q_x = msg.pose.orientation.x
        q_y = msg.pose.orientation.y
        q_z = msg.pose.orientation.z
        q_w = msg.pose.orientation.w
        self.q_pose = np.array([q_x, q_y, q_z, q_w])

        # Extract position from PoseStamped message
        p_x = msg.pose.position.x
        p_y = msg.pose.position.y
        p_z = msg.pose.position.z
        self.p_pose = np.array([p_x, p_y, p_z])

        self.heading = np.arctan2(
            2 * (self.q_pose[3] * self.q_pose[2] + self.q_pose[0] * self.q_pose[1]),
            1 - 2 * (self.q_pose[1]**2 + self.q_pose[2]**2)
        )

        self.pose_list.append(np.array([msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9, *self.q_pose, *self.p_pose]))
    
    def twist_callback(self, msg):
        if self.heading is None:
            self.get_logger().warn("heading is not initialized, skipping twist callback.")
            return

        # linear velocity (body frame)
        v_x = msg.twist.linear.x * np.cos(self.heading) + msg.twist.linear.y * np.sin(self.heading)
        v_y = msg.twist.linear.y * np.cos(self.heading) - msg.twist.linear.x * np.sin(self.heading)
        # v_x = msg.twist.linear.x
        # v_y = msg.twist.linear.y
        v_z = msg.twist.linear.z
        self.v_twist = np.array([v_x, v_y, v_z])

        # angular velocity (body frame)
        w_x = msg.twist.angular.x
        w_y = msg.twist.angular.y
        w_z = msg.twist.angular.z
        self.w_twist = np.array([w_x, w_y, w_z])

        self.twist_list.append(np.array([msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9, *self.w_twist, *self.v_twist]))


    # def get_x_odom(self):
    #     return self.x_odom

    def plot_twist_btw_gt_and_drift(self):

        odom_twist_array = np.array(self.odom_list_twist)   # time, wx, wy, wz, vx, vy, vz
        twist_array = np.array(self.twist_list)             # time, wx, wy, wz, vx, vy, vz

        odom_pose_array = np.array(self.odom_list_pose)    # time, qx, qy, qz, qw, px, py, pz
        pose_array = np.array(self.pose_list)               # time, qx, qy, qz, qw, px, py, pz

        print("odom_twist_array shape:", odom_twist_array.shape)
        print("DRIFT twist_array shape:", twist_array.shape)
        print("odom_pose_array shape:", odom_pose_array.shape)
        print("DRIFT pose_array shape:", pose_array.shape)
        
        plt.subplot(2, 2, 1)
        plt.scatter(twist_array[:, 0], twist_array[:, 4], label='DRIFT Vx in Body Frame', s=5, c='orange')
        plt.scatter(odom_twist_array[:, 0], odom_twist_array[:, 4], label='Ground Truth Vx', s=5, c='blue')
        plt.title('Vx from Ground Truth Twist and DRIFT Twist')
        plt.legend()
        plt.xlabel('Time (s)')
        plt.ylabel('Vx Twist')

        plt.subplot(2, 2, 2)
        plt.scatter(twist_array[:, 0], twist_array[:, 5], label='DRIFT Vy in Body Frame', s=5, c='orange')
        plt.scatter(odom_twist_array[:, 0], odom_twist_array[:, 5], label='Ground Truth Vy', s=5, c='blue')
        plt.title('Vy from Ground Truth Twist and DRIFT Twist')
        plt.legend()
        plt.xlabel('Time (s)')
        plt.ylabel('Vy Twist')

        plt.subplot(2, 2, 3)
        plt.scatter(twist_array[:, 0], twist_array[:, 3], label='DRIFT Wz in Body Frame', s=5, c='orange')
        plt.scatter(odom_twist_array[:, 0], odom_twist_array[:, 3], label='Ground Truth Wz', s=5, c='blue')
        plt.title('Vy btw Ground Truth Twist and DRIFT Twist')
        plt.legend()
        plt.xlabel('Time (s)')
        plt.ylabel('Wz Twist')

        plt.subplot(2, 2, 4)
        plt.scatter(pose_array[:, 4], pose_array[:, 5], label='DRIFT Pose', s=5, c='orange')
        plt.scatter(odom_pose_array[:, 4], odom_pose_array[:, 5], label='Ground Truth Pose', s=5, c='blue')
        plt.title('Pose btw Ground Truth and DRIFT')
        plt.legend()
        plt.xlabel('Position X (m)')
        plt.ylabel('Position Y (m)')

        plt.show()


def main(args=None):
    rclpy.init(args=args)
    Odom_sub = GtOdomSubscriberNode()
    print("start recording")
    for i in range(5000):
        if i % 100 ==0:
    	    print(i)
        rclpy.spin_once(Odom_sub)

    Odom_sub.plot_twist_btw_gt_and_drift()
    # rclpy.spin(Odom_sub)
    Odom_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
