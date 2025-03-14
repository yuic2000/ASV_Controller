#include "../../include/mpc/controller/sim_mpc.h"


Logger sim_mpc(const VectorXd& q0, const VectorXd& p0, const VectorXd& w0, const VectorXd& v0, const Param& param, 
    const std::shared_ptr<MPCCmdPublishNode>& cmd_pub, const std::shared_ptr<GtOdomSubscriberNode>& gtodom_sub) {
    

    const MatrixXd X_ref = param.X_ref;
    const MatrixXd xi_ref = param.xi_ref;
    const int Nsim = param.Nsim;
    const int Nt = param.Nt;
    const double V_c = param.V_c;           // Ocean current velocity
    const double beta_c = param.beta_c;     // Ocean current direction
    const double dt = param.dt;

    // Log the states
    VectorXd x0(13);
    x0 << q0, p0, w0, v0;

    Logger logger(Nsim, Nt);

    logger.X.block(0, 0, logger.X.rows(), 1) = x0;  // initial state

    logger.taken_time = 0.0;


    for (int i = 0; i < Nsim - Nt; ++i) {
        VectorXd X_ref_rt = X_ref.row(i).transpose();
        MatrixXd xi_ref_rt = xi_ref.block(i, 0, Nt, xi_ref.cols());

        auto start_time = std::chrono::high_resolution_clock::now();

        VectorXd u;
        // std::tie(u, solver) = eMPC(x0, X_ref_rt, xi_ref_rt, param, solver);
        u = eMPC(x0, X_ref_rt, xi_ref_rt, param);
        // std::cout << "u_" << i << ": " << u.transpose() << std::endl;

        // Publish thrust commands
        cmd_pub->vrx_thrust_right_pub(u(0));
        cmd_pub->vrx_thrust_left_pub(u(1));
        cmd_pub->mavlink_thrust_pub({u(0), u(1)});

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end_time - start_time;
        // std::cout << "Elapsed time: " << elapsed.count() << " seconds" << std::endl;
        logger.taken_time += elapsed.count();


        //////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////      Use Ground-truth Odometry to update next step       ////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////

        auto listener_start = std::chrono::high_resolution_clock::now();
        rclcpp::spin_some(gtodom_sub);
        VectorXd x_wrld = gtodom_sub->get_x_wrld();
        // std::cout << "x_gtOdom: " << x_wrld.transpose() << std::endl;
        std::cout << "listener: " << std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - listener_start).count() << " seconds" << std::endl;
        x0 = x_wrld;
        std::cout << "x0: " << x0.transpose() << std::endl;

        //////////////////////////////////////////////////////////////////////////////////////////////////


        //////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////      Use Simulation model to update next step            ////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////

        // // Update x_blue four times for simulation frequency, 4*dt
        // Quaterniond quat(x0(3), x0(0), x0(1), x0(2));   // w, x, y, z
        // VectorXd euler_angles = EulerAngles<double, EulerSystemZYX>(quat.normalized().toRotationMatrix()).angles();

        // VectorXd x_blue(12);
        // x_blue << x0.segment(10, 3), x0.segment(7, 3), x0.segment(4, 3), euler_angles.reverse();

        // // Update blue_boat four times for simulation frequency, 4/dt
        // for (int j = 0; j < 4; ++j) {
        //     x_blue += blue_boat(x_blue, u, V_c, beta_c) * dt / 4;
        // }

        // // Convert Euler angles to quaternion
        // euler_angles = x_blue.segment(9, 3);
        // Matrix3d euler_matrix;
        // euler_matrix = AngleAxisd(euler_angles[2], Vector3d::UnitZ()) *
        //         AngleAxisd(euler_angles[1], Vector3d::UnitY()) *
        //         AngleAxisd(euler_angles[0], Vector3d::UnitX());
        
        // Quaterniond quat_euler(euler_matrix);
        // Vector4d quat_vec = quat_euler.coeffs();

        // // Update x0 for the next iteration
        // x0 << quat_vec, x_blue.segment(6, 3), x_blue.segment(3, 3), x_blue.segment(0, 3);
        //////////////////////////////////////////////////////////////////////////////////////////////////


        // Compute Xd and X0
        Matrix4d Xd = Matrix4d::Identity();
        Vector4d seg_ref = X_ref_rt.segment(0, 4);
        Quaterniond q_ref(seg_ref);         // w, x, y, z
        q_ref.normalize();
        Xd.block<3, 3>(0, 0) = q_ref.toRotationMatrix();
        Xd.block<3, 1>(0, 3) = X_ref_rt.segment(4, 3);

        Matrix4d X0 = Matrix4d::Identity();
        Vector4d seg_x0 = x0.segment(0, 4);
        Quaterniond q_x0(seg_x0);           // w, x, y, z
        q_x0.normalize();
        X0.block<3, 3>(0, 0) = q_x0.toRotationMatrix();
        X0.block<3, 1>(0, 3) = x0.segment(4, 3);

        // Compute Xerr and XXerr
        Matrix4d Xerr = (X0.inverse() * Xd).log();
        Matrix4d XXerr = X0.inverse() * Xd;

        // Compute x00
        VectorXd x00(6);
        x00.segment(0, 3) << Xerr(2, 1), Xerr(0, 2), Xerr(1, 0);
        x00.segment(3, 3) = Xerr.block<3, 1>(0, 3);



        VectorXd err2(4);
        err2 << std::sqrt((XXerr.block<3, 3>(0, 0).log().array().square().sum()) / 2), XXerr.block<3, 1>(0, 3);

        // update logger
        logger.X.block(0, i+1, logger.X.rows(), 1) = x0;
        logger.U.block(0, i, logger.U.rows(), 1) = u;
        logger.Err.block(0, i, logger.Err.rows(), 1) = x00;
        logger.Err2.block(0, i, logger.Err2.rows(), 1) = err2;
    }

    return logger;
}