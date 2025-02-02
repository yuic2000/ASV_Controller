import numpy as np
from scipy.spatial.transform import Rotation
from scipy.linalg import expm
import pickle

from mpc_python.sim_mpc import sim_mpc
from mpc_python.src.utils.LieGroup import skew
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

    x0_ref = param.x0_ref
    xid_ref = param.xid_ref
    X_ref = param.X_ref
    xi_ref = param.xi_ref

    X = np.eye(4)

    for i in range(param.Nsim):
        xid_ref_rt = xid_ref.copy()
        
        if param.mode == 1:  # zigzag maneuver
            xid_ref_rt[2] = 0.1 * np.cos(i/50)      # 100: 20 Hz, 50: 10 Hz
        
        Xi = np.zeros((4, 4))
        Xi[:3, :3] = skew(xid_ref_rt[:3])
        Xi[:3, 3] = xid_ref_rt[3:]
        
        X = X @ expm(Xi * param.dt)
        new_quat = Rotation.from_matrix(X[:3, :3]).as_quat()
        X_ref = np.vstack([X_ref, np.concatenate([new_quat, X[:3, 3]])])
        xi_ref = np.vstack([xi_ref, xid_ref_rt])

    param.X_ref = X_ref
    param.xi_ref = xi_ref

    # Run simulation
    Logger_eMPC_list = []
    Logger_eMPC = Logger(param.Nsim)

    # Initialize MPC node, to be tested
    rclpy.init(args=args)

    # Subscribe to ground truth odometry
    gtOdom_sub = GtOdomSubscriberNode()
    gtOdom_sub.get_logger().info("Hi: gtodom_sub is running")

    # Publish thrust commands
    cmd_pub = MPCCmdPulishNode()
    cmd_pub.get_logger().info("Hi: cmd_pub is running")

    # Create a multi-threaded executor
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(gtOdom_sub)
    executor.add_node(cmd_pub)

    # rclpy.spin(cmd_pub)

    rclpy.spin_once(gtOdom_sub, timeout_sec=0.1)
    rclpy.spin_once(cmd_pub, timeout_sec=0.1)

    for k in range(Ns):
        # Setting initial states
        q0 = Rotation.from_euler('xyz', [0, 0, (np.random.rand()-0.5)*0.5*np.pi]).as_quat()
        p0 = np.zeros(3)
        p0[:2] += (np.random.rand(2)-1)*5
        p0[1] += 2.5
        w0 = (np.random.rand(3) - 0.5) * 0
        v0 = (np.random.rand(3) - 0.5) * 0
        v0[0] = 0.5
        Logger_eMPC = sim_mpc(q0, p0, w0, v0, param, cmd_pub, gtOdom_sub)
        Logger_eMPC_list.append(Logger_eMPC)


    cmd_pub.left_thrust_command_pub(0.0)
    cmd_pub.right_thrust_command_pub(0.0)


    # Save Logger_eMPC_list to a file
    if param.mode == 1:       # mode 1: zigzag, 
        with open('zigzag_data.pkl', 'wb') as f:
            pickle.dump(Logger_eMPC_list, f)
    else:               # mode 2: circling
        with open('circling_data.pkl', 'wb') as f:
            pickle.dump(Logger_eMPC_list, f)

    plot_mpc(Logger_eMPC_list, param.X_ref, param.dt)

    # Destroy nodes and shutdown
    gtOdom_sub.destroy_node()
    cmd_pub.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()