import numpy as np
import pickle

from mpc_python.sim_mpc import sim_mpc
from mpc_python.src.param import *
from mpc_python.src.plot import *

import rclpy
from mpc_python.communication.mpc_cmd_pub import *
from mpc_python.communication.gtodom_sub import *


def main(args=None): 
    # Initialize parameters
    np.random.seed(2)
    Ns = 1
    param = Param()             # Initialize parameters in param.py

    waypoint_by_twist = False   # True: set waypoints by twists, False: set waypoints by positions


    # Initialize MPC node
    rclpy.init(args=args)

    # Subscribe to ground truth odometry
    gtOdom_sub = GtOdomSubscriberNode()
    # Publish thrust commands
    cmd_pub = MPCCmdPulishNode()

    # # Create a multi-threaded executor
    # executor = rclpy.executors.MultiThreadedExecutor()
    # executor.add_node(gtOdom_sub)
    # executor.add_node(cmd_pub)

    # Get initial states if using ground truth odometry
    rclpy.spin_once(gtOdom_sub)
    x_init = gtOdom_sub.get_x_wrld()

    param.set_desired_posistion(x_init)  # Set desired position
    if waypoint_by_twist:
        print('Generating waypoints by twists')
        param.get_ref_trajectory()           # Get reference trajectory for zigzag or circling by twists
    
    else:
        print('Generating waypoints by positions')
        if param.mode == 1:                     # mode 1: zigzag, mode 2: circling
            param.generate_zigzag_waypoints()   # zigzag, directly set zigzag waypoints
        elif param.mode == 2: 
            param.generate_circle_waypoints()   # circling, directly set circle waypoints
        param.waypoints2twists()                # convert waypoints to referenced twists


    # Run simulation
    Logger_eMPC = Logger(param.Nsim, param.Nt)

    #####################################################################################
    # Setting initial states, for using boat model
    #####################################################################################
    # q0 = Rotation.from_euler('xyz', [0, 0, (np.random.rand()-0.5)*0.5*np.pi]).as_quat()
    # p0 = np.zeros(3)
    # p0[:2] += (np.random.rand(2)-1) * 2
    # p0[1] += 1.5
    # w0 = (np.random.rand(3) - 0.5) * 0
    # v0 = (np.random.rand(3) - 0.5) * 0
    # v0[0] = 0.5
    #####################################################################################
    # End of using boat model
    #####################################################################################
    
    #####################################################################################
    # Read initial states, for using ground truth odometry
    #####################################################################################
    # rclpy.spin_once(gtOdom_sub)
    # x_init = gtOdom_sub.get_x_wrld()
    q0 = x_init[0:4]
    p0 = x_init[4:7]
    w0 = x_init[7:10]
    v0 = x_init[10:13]
    #####################################################################################
    # End of using ground truth odometry
    #####################################################################################
    Logger_eMPC = sim_mpc(q0, p0, w0, v0, param, cmd_pub, gtOdom_sub)

    cmd_pub.vrx_thrust_left_pub(0.0)
    cmd_pub.vrx_thrust_right_pub(0.0)
    cmd_pub.mavlink_thrust_pub([0.0, 0.0])
    
    # Destroy nodes and shutdown
    gtOdom_sub.destroy_node()
    cmd_pub.destroy_node()
    rclpy.shutdown()


    # Save Logger_eMPC to a file
    if param.mode == 1:       # mode 1: zigzag, 
        with open('zigzag_data.pkl', 'wb') as f:
            pickle.dump(Logger_eMPC, f)
    else:               # mode 2: circling
        with open('circling_data.pkl', 'wb') as f:
            pickle.dump(Logger_eMPC, f)

    plot_mpc(Logger_eMPC, param.X_ref, param.dt)


if __name__ == '__main__':
    main()