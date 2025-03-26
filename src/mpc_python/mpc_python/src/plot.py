import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

def plot_mpc(logger, X_ref, dt):
    axis_font_size = 12
    x_lim = [0, 60]

    ################################################################
    # Error plots
    ################################################################
    fig1, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        
    t = np.arange(0, logger.Err2.shape[1] * dt, dt)
    
    # Orientation error
    ax1.plot(t, logger.Err2[0, :], linewidth=0.5)
    # Position error
    pos_error = np.linalg.norm(logger.Err2[1:4, :], axis=0)
    ax2.plot(t, pos_error, linewidth=0.5)
    
    # Orientation error
    ax1.set_title("MPC - Orientation error", fontsize=axis_font_size)
    ax1.grid(True)
    ax1.set_xlim(x_lim)
    ax1.set_ylabel(r"$\|Log(R_d^{-1}R)\|$")
    ax1.set_xlabel("Time (s)")
    
    # Position error
    ax2.set_title("MPC - Position error", fontsize=axis_font_size)
    ax2.grid(True)
    ax2.set_xlim(x_lim)
    ax2.set_ylabel(r"$\|p-p_d\|$")
    ax2.set_xlabel("Time (s)")
    
    plt.tight_layout()
    ################################################################
    # End of error plots
    ################################################################
    
    ################################################################
    # Trajectory plot
    ################################################################
    fig2 = plt.figure(figsize=(10, 8))
    ax = fig2.add_subplot(111)
    
    # Plot MPC trajectory
    ax.plot(logger.X[4, :], logger.X[5, :], 
            color='r', linestyle='-.', linewidth=2, label='MPC')
    
    # Plot reference trajectory
    ax.plot(X_ref[:, 4], X_ref[:, 5], 'k-', 
            linewidth=3, label='Reference')
    
    ax.set_aspect('equal')
    ax.grid(True)
    ax.legend()
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ################################################################
    # End of trajectory plot
    ################################################################


    ################################################################
    # Time taken
    ################################################################
    print(f'simulation: {logger.taken_time} sec')
    # for idx, logger in enumerate(Logger_list):
    #     print(f'{idx+1} simulation: {logger.taken_time} sec')
    ################################################################
    # End of time taken
    ################################################################
    
    plt.show()
    plt.close()

def plot_euler(ref1, ref2, ref_1_i, ref_2_i):
        fig1, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 5))

        # Extract Euler angles from quaternions
        euler_ref1 = np.array([Rotation.from_quat(row[:4]).as_euler('xyz') for row in ref1])
        euler_ref2 = np.array([Rotation.from_quat(row[:4]).as_euler('xyz') for row in ref2])

        # Time axis
        t = np.arange(ref1.shape[0])

        # Plot yaw (z-axis rotation)
        ax1.plot(t, euler_ref1[:, 2], label='Ref1', color='b')
        ax1.plot(t, euler_ref2[:, 2], label='Ref2', color='r')
        ax1.set_title('Yaw (z-axis)')
        ax1.set_xlabel('Time')
        ax1.set_ylabel('Angle (rad)')
        ax1.grid(True)
        ax1.legend()

        ax2.plot(t, ref_1_i[:, 2], label='Ref1', color='b')     # 0.1 rad/s
        ax2.plot(t, ref_2_i[:, 2], label='Ref2', color='r', linestyle='--')
        ax2.set_title('anguler velocity z')
        ax2.set_xlabel('Time')
        ax2.set_ylabel('rad/s')
        ax2.grid(True)
        ax2.legend()

        plt.tight_layout()


def plot_linear_twist(ref1, ref2):
        fig2, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 5))

        # Time axis
        t = np.arange(ref1.shape[0])
        ax1.plot(t, ref1[:, 3], label='Ref1', color='b')       # 0.5 m/s
        ax1.plot(t, ref2[:, 3], label='Ref2', color='r')
        ax1.set_title('linear velocity x')
        ax1.set_xlabel('Time')
        ax1.set_ylabel('m/s')
        ax1.grid(True)
        ax1.legend()

        ax2.plot(t, ref1[:, 4], label='Ref1', color='b')       # 0 m/s
        ax2.plot(t, ref2[:, 4], label='Ref2', color='r')
        ax2.set_title('linear velocity y')
        ax2.set_xlabel('Time')
        ax2.set_ylabel('m/s')
        ax2.grid(True)
        ax2.legend()

        plt.tight_layout()
        plt.show()
        plt.close()