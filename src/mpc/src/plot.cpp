#include "../include/mpc/plot.h"

namespace plt = matplotlibcpp;

void plot_mpc(const Logger& logger, const Param& param) {
    const double dt = param.dt;
    const int Nt = param.Nt;
    const int Nsim = param.Nsim;
    const MatrixXd& X_ref = param.X_ref;

    // std::vector<double> x_lim = {0, 60};
    int axis_font_size = 12;

    //////////////////////////////////////////////////////////////////////////////////////
    // Error plots
    //////////////////////////////////////////////////////////////////////////////////////
    plt::figure_size(1200, 500);
    plt::subplot(1, 2, 1);
    
    // Timestamp
    std::vector<double> t(logger.Err2.cols());
    for (uint i = 0; i < t.size(); ++i) {
        t[i] = i * dt;
    }

    // Orientation error
    std::vector<double> orientation_error(logger.Err2.cols());
    for (uint i = 0; i < logger.Err2.cols(); ++i) {
        orientation_error[i] = (logger.Err2(0, i));
    }
    plt::plot(t, orientation_error, {{"linewidth", "0.5"}});

    // Orientation error plot settings
    plt::title("MPC - Orientation error", {{"fontsize", std::to_string(axis_font_size)}});
    plt::grid(true);
    // plt::xlim(x_lim[0], x_lim[1]);
    plt::ylabel("$\\|Log(R_d^{-1}R)\\|$");
    plt::xlabel("Time (s)");

    plt::subplot(1, 2, 2);

    // Position error
    std::vector<double> pos_error(logger.Err2.cols());
    for (uint i = 0; i < logger.Err2.cols(); ++i) {
        pos_error[i] = std::sqrt(std::pow(logger.Err2(1, i), 2) + std::pow(logger.Err2(2, i), 2) + std::pow(logger.Err2(3, i), 2));
    }
    plt::plot(t, pos_error, {{"linewidth", "0.5"}});

    // Position error plot settings
    plt::title("MPC - Position error", {{"fontsize", std::to_string(axis_font_size)}});
    plt::grid(true);
    // plt::xlim(x_lim[0], x_lim[1]);
    plt::ylabel("$\\|p-p_d\\|$");
    plt::xlabel("Time (s)");

    plt::tight_layout();
    //////////////////////////////////////////////////////////////////////////////////////
    // End of error plots
    //////////////////////////////////////////////////////////////////////////////////////


    //////////////////////////////////////////////////////////////////////////////////////
    // Trajectory plot
    //////////////////////////////////////////////////////////////////////////////////////
    plt::figure_size(1000, 800);
    
    // Real trajectory
    std::vector<double> x(logger.Err2.cols());
    std::vector<double> y(logger.Err2.cols());
    for (int i = 0; i < logger.Err2.cols(); ++i) {
        x[i] = (logger.X(4, i));
        y[i] = (logger.X(5, i));
    }

    plt::plot(std::vector<double>{x[0]}, std::vector<double>{y[0]}, "bo-", {{"label", "Start"}, {"markersize", "10"}});
    plt::plot(std::vector<double>{x[logger.Err2.cols() - 1]}, std::vector<double>{y[logger.Err2.cols() - 1]}, "k*", {{"label", "End"}, {"markersize", "10"}});
    plt::plot(x, y, "r-.", {{"linewidth", "3"}, {"label", "MPC"}});

    // Reference trajectory
    std::vector<double> x_ref(X_ref.col(4).data(), X_ref.col(4).data() + X_ref.rows());
    std::vector<double> y_ref(X_ref.col(5).data(), X_ref.col(5).data() + X_ref.rows());
    
    plt::plot(std::vector<double>{x_ref[0]}, std::vector<double>{y_ref[0]}, "bo-", {{"markersize", "10"}});
    plt::plot(std::vector<double>{x_ref[X_ref.rows() - 1]}, std::vector<double>{y_ref[X_ref.rows() - 1]}, "k*", {{"markersize", "10"}});
    plt::plot(x_ref, y_ref, "k-", {{"linewidth", "2"}, {"label", "Reference"}});

    plt::axis("equal");
    plt::grid(true);
    plt::legend();
    plt::xlabel("x");
    plt::ylabel("y");

    //////////////////////////////////////////////////////////////////////////////////////
    // End of trajectory plot
    //////////////////////////////////////////////////////////////////////////////////////


    std::cout << " simulation: " << logger.taken_time << " sec" << std::endl;

    plt::show();
    plt::close();
}