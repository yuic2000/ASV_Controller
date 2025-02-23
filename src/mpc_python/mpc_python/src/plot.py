import matplotlib.pyplot as plt
import numpy as np

def plot_mpc(Logger_list, X_ref, dt):
    axis_font_size = 12
    x_lim = [0, 60]

    # Error plots
    fig1, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    for logger in Logger_list:
        # print(f'X: {logger.X.shape}')           # 13, 1201
        # print(f'Err: {logger.Err.shape}')       # 6, 1200
        # print(f'U: {logger.U.shape}')           # 2, 1200
        # print(f'Err2: {logger.Err2.shape}')     # 4, 1200
        
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
    
    # Trajectory plot
    fig2 = plt.figure(figsize=(10, 8))
    ax = fig2.add_subplot(111)
    
    # Plot the last trajectory result
    logger = Logger_list[-1]
    # Plot MPC trajectory
    ax.plot(logger.X[4, :1200], logger.X[5, :1200], 
            color='r', linestyle='-.', linewidth=2, label='MPC')
    
    # Plot reference trajectory
    ax.plot(X_ref[:1200, 4], X_ref[:1200, 5], 'k-', 
            linewidth=3, label='Reference')
    
    # Plot all trajectory results
#     for logger in Logger_list:
#         # Plot MPC trajectory
#         ax.plot(logger.X[4, :1200], logger.X[5, :1200], 
#             color='r', linestyle='-.', linewidth=2, label='MPC')
    
#         # Plot reference trajectory
#         ax.plot(X_ref[:1200, 4], X_ref[:1200, 5], 'k-', 
#             linewidth=3, label='Reference')
    
    ax.set_aspect('equal')
    ax.grid(True)
    ax.legend()
    ax.set_xlabel('x')
    ax.set_ylabel('y')

    # Plot control inputs
    # fig3 = plt.figure(figsize=(10, 8))
    # ax = fig3.add_subplot(111)
    # logger = Logger_list[-1]

    # t = np.arange(0, logger.U.shape[1] * dt, dt)

    # ax.plot(t, logger.U[0, :1200], 'r-', linewidth=2, label='Right thrust')
    # ax.plot(t, logger.U[1, :1200], 'b-', linewidth=2, label='Left thrust')

    # ax.set_title("Propeller thrusts", fontsize=axis_font_size)
    # ax.grid(True)
    # ax.set_ylabel("Thrust (N)")
    # ax.set_xlabel("Time (s)")

    for idx, logger in enumerate(Logger_list):
        print(f'{idx+1} simulation: {logger.taken_time} sec')
    
    plt.show()
    plt.close()