import numpy as np
import pickle

from mpc_python.sim_mpc import sim_mpc
from mpc_python.src.param import *
from mpc_python.src.plot import plot_mpc

import rclpy
from mpc_python.communication.mpc_cmd_pub import *
from mpc_python.communication.gtodom_sub import *


def main(args=None): 
    # Initialize parameters
    np.random.seed(2)
    Ns = 1
    param = Param()             # Initialize parameters in param.py

    param.get_ref_trajectory()  # Get reference trajectory


    # Run simulation
    Logger_eMPC_list = []
    Logger_eMPC = Logger(param.Nsim, param.Nt)

    # Initialize MPC node, to be tested
    rclpy.init(args=args)

    # Subscribe to ground truth odometry
    gtOdom_sub = GtOdomSubscriberNode()
    gtOdom_sub.get_logger().info("Ground-truth Odometry subscriber is running")

    # Publish thrust commands
    cmd_pub = MPCCmdPulishNode()
    cmd_pub.get_logger().info("Command publiser node is running")

    # Create a multi-threaded executor
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(gtOdom_sub)
    executor.add_node(cmd_pub)

    for k in range(Ns):
        # Setting initial states, for using otter true model
        # q0 = Rotation.from_euler('xyz', [0, 0, (np.random.rand()-0.5)*0.5*np.pi]).as_quat()
        # p0 = np.zeros(3)
        # p0[:2] += (np.random.rand(2)-1) * 2
        # p0[1] += 1.5
        # w0 = (np.random.rand(3) - 0.5) * 0
        # v0 = (np.random.rand(3) - 0.5) * 0
        # v0[0] = 0.5
        
        # Read initial states, for using ground truth odometry
        rclpy.spin_once(gtOdom_sub)
        x_wrld = gtOdom_sub.get_x_wrld()

        q0 = x_wrld[0:4]
        p0 = x_wrld[4:7]
        w0 = x_wrld[7:10]
        v0 = x_wrld[10:13]

        Logger_eMPC = sim_mpc(q0, p0, w0, v0, param, cmd_pub, gtOdom_sub)
        Logger_eMPC_list.append(Logger_eMPC)


    cmd_pub.vrx_thrust_left_pub(0.0)
    cmd_pub.vrx_thrust_right_pub(0.0)
    cmd_pub.mavlink_thrust_pub([0.0, 0.0])
    
    # Destroy nodes and shutdown
    gtOdom_sub.destroy_node()
    cmd_pub.destroy_node()
    rclpy.shutdown()


    # Save Logger_eMPC_list to a file
    if param.mode == 1:       # mode 1: zigzag, 
        with open('zigzag_data.pkl', 'wb') as f:
            pickle.dump(Logger_eMPC_list, f)
    else:               # mode 2: circling
        with open('circling_data.pkl', 'wb') as f:
            pickle.dump(Logger_eMPC_list, f)

    plot_mpc(Logger_eMPC_list, param.X_ref, param.dt)


if __name__ == '__main__':
    main()