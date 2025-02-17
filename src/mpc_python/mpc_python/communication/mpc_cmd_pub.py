#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

class MPCCmdPulishNode(Node):
    def __init__(self):
        super().__init__('mpc_thrust_cmd_publisher')
        self.get_logger().info("mpc_node is running")

        self.vrx_thrust_pub_l_ = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.vrx_thrust_pub_r_ = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.mavlink_thrust_pub_ = self.create_publisher(Float64MultiArray, '/mavlink/thrust_commands', 10)

    def vrx_thrust_left_pub(self, thrust):
        msg = Float64()
        msg.data = float(thrust)
        self.vrx_thrust_pub_l_.publish(msg)
        # self.get_logger().info("Publishing left propeller thrust: '%.2f'" % msg.data)

    def vrx_thrust_right_pub(self, thrust):
        msg = Float64()
        msg.data = float(thrust)
        self.vrx_thrust_pub_r_.publish(msg)
        # self.get_logger().info("Publishing right propeller thrust: '%.2f'" % msg.data)

    def mavlink_thrust_pub(self, thrust):
        msg = Float64MultiArray()
        thrust = [float(thrust[0]), float(thrust[1])]
        msg.data = thrust
        self.mavlink_thrust_pub_.publish(msg)
        self.get_logger().info(f"right: {msg.data[0]:.2f}, left: {msg.data[1]:.2f}")


def main(args=None):
    rclpy.init(args=args)

    mpc_node = MPCCmdPulishNode()

    rclpy.spin(mpc_node)

    mpc_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
