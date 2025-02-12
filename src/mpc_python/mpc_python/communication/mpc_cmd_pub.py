#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class MPCCmdPulishNode(Node):
    def __init__(self):
        super().__init__('mpc_thrust_cmd_publisher')
        self.get_logger().info("mpc_node is running")

        self.cmd_thrust_pub_l_ = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.cmd_thrust_pub_r_ = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

    
    def left_thrust_command_pub(self, thrust):
        msg = Float64()
        msg.data = thrust
        self.cmd_thrust_pub_l_.publish(msg)
        # self.get_logger().info("Publishing left propeller thrust: '%f'" % msg.data)

    def right_thrust_command_pub(self, thrust):
        msg = Float64()
        msg.data = thrust
        self.cmd_thrust_pub_r_.publish(msg)
        # self.get_logger().info("Publishing right propeller thrust: '%f'" % msg.data)

def main(args=None):
    rclpy.init(args=args)

    mpc_node = MPCCmdPulishNode()

    rclpy.spin(mpc_node)

    mpc_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
