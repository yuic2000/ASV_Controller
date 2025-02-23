#include "../include/mpc/plot.h"

namespace plt = matplotlibcpp;

void plot_mpc(const std::vector<Logger>& Logger_list, const Param& param) {
    const double dt = param.dt;
    const int Nt = param.Nt;
    const int Nsim = param.Nsim;
    const MatrixXd& X_ref = param.X_ref;

    std::vector<double> x_lim = {0, 60};
    int axis_font_size = 12;

    // Error plots
    plt::figure_size(1200, 500);
    plt::subplot(1, 2, 1);
    
    for (const auto& logger : Logger_list) {
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
    }

    // Orientation error plot settings
    plt::title("MPC - Orientation error", {{"fontsize", std::to_string(axis_font_size)}});
    plt::grid(true);
    plt::xlim(x_lim[0], x_lim[1]);
    plt::ylabel("$\\|Log(R_d^{-1}R)\\|$");
    plt::xlabel("Time (s)");

    plt::subplot(1, 2, 2);
    
    for (const auto& logger : Logger_list) {
        std::vector<double> t(logger.Err2.cols());
        for (size_t i = 0; i < t.size(); ++i) {
            t[i] = i * dt;
        }

        // Position error
        std::vector<double> pos_error(logger.Err2.cols());
        for (uint i = 0; i < logger.Err2.cols(); ++i) {
            pos_error[i] = std::sqrt(std::pow(logger.Err2(1, i), 2) + std::pow(logger.Err2(2, i), 2) + std::pow(logger.Err2(3, i), 2));
        }
        plt::plot(t, pos_error, {{"linewidth", "0.5"}});
    }

    // Position error plot settings
    plt::title("MPC - Position error", {{"fontsize", std::to_string(axis_font_size)}});
    plt::grid(true);
    plt::xlim(x_lim[0], x_lim[1]);
    plt::ylabel("$\\|p-p_d\\|$");
    plt::xlabel("Time (s)");

    plt::tight_layout();

    // Trajectory plot
    plt::figure_size(1000, 800);
    
    // Plot the last trajectory result
    const auto& logger = Logger_list.back();
            // Orientation error
    std::vector<double> x(logger.Err2.cols());
    std::vector<double> y(logger.Err2.cols());
    for (int i = 0; i < logger.Err2.cols(); ++i) {
        x[i] = (logger.X(4, i));
        y[i] = (logger.X(5, i));
    }

    plt::plot(x, y, "r-.", {{"linewidth", "2"}, {"label", "MPC"}});

    // Plot reference trajectory
    std::vector<double> x_ref(X_ref.col(4).data(), X_ref.col(4).data() + (Nsim - Nt + 1));
    std::vector<double> y_ref(X_ref.col(5).data(), X_ref.col(5).data() + (Nsim - Nt + 1));
    plt::plot(x_ref, y_ref, "k-", {{"linewidth", "3"}, {"label", "Reference"}});

    plt::axis("equal");
    plt::grid(true);
    plt::legend();
    plt::xlabel("x");
    plt::ylabel("y");

    // Print simulation time
    for (size_t idx = 0; idx < Logger_list.size(); ++idx) {
        std::cout << (idx + 1) << " simulation: " << Logger_list[idx].taken_time << " sec" << std::endl;
    }

    plt::show();
    plt::close();
}