#!/usr/bin/env python3
#
import numpy as np, time, inspect
import rclpy
from rclpy.node          import Node
from geometry_msgs.msg   import PoseStamped
from nav_msgs.msg        import Path
from nav_msgs.msg        import Odometry
from std_msgs.msg        import Float64, Float64MultiArray
from scipy.spatial.transform import Rotation

# from mpc_python.communication.mpc_cmd_pub import *
from mpc_python.src.param import *
from mpc_python.src.plot import plot_mpc
from mpc_python.src.controller.eMPC import eMPC
from mpc_python.src.utils.LieGroup import *

print("Using Param from: ", inspect.getfile(Param))

class MPC(Node):
    def __init__(self):
        super().__init__("mpc_node")

        # ─────── Load parameters ───────
        try:
            self.param = Param()
        except Exception as e:
            raise RuntimeError(f"Error loading parameters: {e}")

        self.solver = {'init': 0}
        self.initialized_wp = True     # False: using path callback, True: using Zigzag/Circling waypoints

        # ───────── state ─────────
        self.path           = []      # list[ np.array([x,y]) ]
        self.i_wp           = 0       # current waypoint index
        self.reached_final  = False
        self.X = []
        self.U = []
        self.Err = []
        self.Err2 = []

        # ───────── subs & pubs ─────────
        self.create_subscription(
            Path,
            self.param.path_topic,
            self.cb_path,
            10)

        self.create_subscription(
            Odometry,
            self.param.pose_topic,
            self.cb_odom,
            10)

        self.vrx_thrust_pub_l_ = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.vrx_thrust_pub_r_ = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.mavlink_thrust_pub_ = self.create_publisher(Float64MultiArray, '/mavlink/thrust_commands', 10)

        self.get_logger().info('MPC path follower ready ✔')

    # ───────── callbacks ─────────
    def cb_path(self, msg: Path):
        """Store the entire path as a flat list of XY waypoints."""
        self.path = [np.array([p.pose.position.x,
                               p.pose.position.y]) for p in msg.poses]
        self.Nsim = len(self.path)
        
        self.param.X_ref = np.empty((0, 7))  # reset reference trajectory
        self.param.xi_ref = np.empty((0, 6))  # reset twist trajectory
        
        for i in range(self.Nsim - 1):
            p1 = self.path[i]
            p2 = self.path[i + 1]
            dx, dy = p2[0] - p1[0], p2[1] - p1[1]
            yaw = np.arctan2(dy, dx)    # radians, in world frame
            quaternion = Rotation.from_euler('xyz', [0, 0, yaw]).as_quat()

            self.param.X_ref = np.vstack([self.param.X_ref, np.flatten([quaternion, [p1, 0]])])

        self.param.waypoints2twist()        # generate twist trajectory
        self.logger = Logger(self.Nsim, self.param.Nt)

        self.i_wp = 0
        self.reached_final = False
        self.get_logger().info(f'Received path with {self.Nsim} waypoints')

    def cb_odom(self, msg: Odometry):
        # Boat states: return self.x_wrld (Orientation, Position, Angular Velocity, Velocity)
        self.odom_transform(msg)

        # ─────── Initializee Zigzag/Circling Waypoints when not using path callback ───────
        if self.initialized_wp and self.i_wp==0:
            self.param.set_desired_posistion(self.x_wrld)
            # self.param.X_ref = np.empty((0, 7))  # reset reference trajectory
            # self.param.xi_ref = np.empty((0, 6))
            self.param.generate_zigzag_waypoints()
            # self.param.generate_circle_waypoints()
            self.param.waypoints2twists()        # generate twist trajectory
            self.Nsim = len(self.param.X_ref)
            self.logger = Logger(self.Nsim, self.param.Nt)
            self.initialized_wp = False
        
        # ─────── Checked if path is given or reach final waypoint ───────
        if self.param.X_ref.size == 0 or self.reached_final:
            self.get_logger().info("No path to follow or path already completed.")
            return  # nothing to track
       
        # ───────── Current Waypoint ─────────
        X_ref_rt = self.param.X_ref[self.i_wp]
        xi_ref_rt = self.param.xi_ref[self.i_wp : self.i_wp + self.param.Nt]
        self.i_wp += 1

        # ───────── Check if reached the final waypoint ─────────
        if self.i_wp >= self.Nsim - self.param.Nt:
            self.reached_final = True  # finished
            self.stop_boat()
            self.get_logger().info("Finished path 🎉")
            self.logger.X = np.array(self.X).T
            self.logger.U = np.array(self.U).T
            self.logger.Err = np.array(self.Err).T
            self.logger.Err2 = np.array(self.Err2).T
            plot_mpc(self.logger, self.param.X_ref, self.param.dt)
            return

        # ───────── MPC ─────────
        start_time = time.time()
        u, self.solver = eMPC(self.x_wrld, X_ref_rt, xi_ref_rt, self.param, self.solver)    # u[0]: right, u[1]: left

        self.vrx_thrust_pub_r_.publish(Float64(data=float(u[0])))       # right
        self.vrx_thrust_pub_l_.publish(Float64(data=float(u[1])))       # left
        self.mavlink_thrust_pub_.publish(Float64MultiArray(data=[float(u[0]), float(u[1])]))    # right, left
        self.get_logger().info(f"right: {u[0]:7.2f}, left: {u[1]:7.2f}")
        
        # ───────── Save in Logger ─────────
        Xd = np.eye(4)
        Xd[0:3, 0:3] = quat2rotm(X_ref_rt[0:4])
        Xd[0:3, 3] = X_ref_rt[4:7]

        X0 = np.eye(4)
        X0[0:3, 0:3] = quat2rotm(self.x_wrld[0:4])
        X0[0:3, 3] = self.x_wrld[4:7]

        Xerr = logm(np.linalg.inv(X0).dot(Xd))
        XXerr = np.linalg.inv(X0).dot(Xd)

        x00 = np.zeros(6)
        x00[0:3] = [Xerr[2, 1], Xerr[0, 2], Xerr[1, 0]]
        x00[3:6] = Xerr[0:3, 3]

        self.X.append(self.x_wrld)
        self.U.append(u)
        self.Err.append(x00)
        self.Err2.append([np.sqrt(np.sum(logm(XXerr[:3, :3])**2) / 2), *XXerr[0:3, 3]])
        self.logger.taken_time += (time.time() - start_time)



    # ───────── helper ─────────
    def stop_boat(self):
        self.vrx_thrust_pub_l_.publish(Float64(data=0.0))
        self.vrx_thrust_pub_r_.publish(Float64(data=0.0))
        self.mavlink_thrust_pub_.publish(Float64MultiArray(data=[0.0, 0.0]))
        self.get_logger().info("Boat stopped")
    
    def odom_transform(self, msg):
        # position (world frame)
        p_x = msg.pose.pose.position.x
        p_y = msg.pose.pose.position.y
        p_z = msg.pose.pose.position.z
        p_odom = np.array([p_x, p_y, p_z])

        # orientation (world frame)
        q_x = msg.pose.pose.orientation.x
        q_y = msg.pose.pose.orientation.y
        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w
        q_odom = np.array([q_x, q_y, q_z, q_w])

        # linear velocity (body frame)
        v_x = msg.twist.twist.linear.x
        v_y = msg.twist.twist.linear.y
        v_z = msg.twist.twist.linear.z
        v_body = np.array([v_x, v_y, v_z])

        # angular velocity (body frame)
        w_x = msg.twist.twist.angular.x
        w_y = msg.twist.twist.angular.y
        w_z = msg.twist.twist.angular.z
        w_body = np.array([w_x, w_y, w_z])

        self.x_wrld = np.concatenate([q_odom, p_odom, w_body, v_body], axis=None)


def main(args=None):
    rclpy.init(args=args)
    node = MPC()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()