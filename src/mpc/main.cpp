#include <random>
#include <fstream>
#include <vector>

#include "include/mpc/controller/sim_mpc.h"
#include "include/mpc/plot.h"
#include "include/mpc/param_class.h"
#include "include/mpc/communication/mpc_cmd_pub.hpp"
#include "include/mpc/communication/gtodom_sub.hpp"

using namespace std;
using namespace Eigen;


int main(int argc, char **argv) {
    std::mt19937 gen(2); // Mersenne Twister generator
    std::uniform_real_distribution<double> dis(-0.125 * M_PI, 0.125 * M_PI); // Distribution for angles between -0.25*pi and 0.25*pi
    Eigen::setNbThreads(6); // Use 4 threads


    Param param;
    int mode = param.mode;              // 1: zigzag, 2: circling
    int Nsim = param.Nsim;              // Total simulation time + Planning time
    int Nt = param.Nt;                  // MPC planning time

    param.set_reference_trajectory();

    rclcpp::init(argc, argv);
    
    // Subscribe to ground truth odometry
    auto gtodom_sub = std::make_shared<GtOdomSubscriberNode>();

    // Publish thrust commands
    auto cmd_pub = std::make_shared<MPCCmdPublishNode>();

    // Get initial states from Ground-truth Odometry
    VectorXd x_init;
    while (x_init.size() == 0) {
        std::cout << "Waiting for ground truth odometry..." << std::endl;
        rclcpp::spin_some(gtodom_sub);

        x_init = gtodom_sub->get_x_wrld();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    Logger Logger_eMPC(Nsim, Nt);
    ///////////////////////////////////////////////////////////////////////////////////
    // setting initial states randomly
    // q0: randomly rotated -0.25*pi to 0.25*pi along z-axis
    // Quaterniond q0(AngleAxisd(dis(gen), Vector3d::UnitZ()));    // x, y, z, w
    // Vector3d p0 = Vector3d::Zero();
    // p0.head<2>() += (Vector2d::Random() - Vector2d::Ones()) * 2;
    // p0(1) += 1.5;
    // Vector3d w0 = (Vector3d::Random() - Vector3d::Ones()) * 0.0;
    // Vector3d v0 = (Vector3d::Random() - Vector3d::Ones()) * 0.0;;
    // v0(0) = 0.5;
    // VectorXd q0_vec(4);
    // q0_vec << q0.coeffs();
    // setting initial states from python random values
    // Vector4d q0_vec;
    // q0_vec << 0.0, 0.0, -0.05024832, 0.99873676;    // x, y, z, w 
    // Vector3d p0;
    // p0 << -4.87036884, 0.24831239, 0.0;
    // Vector3d w0;
    // w0 << -0.0, -0.0, -0.0;
    // Vector3d v0;
    // v0 << 0.5, 0.0, -0.0;
    ///////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////
    // setting initial states from Ground-tryuth Odometry
    // rclcpp::spin_some(gtodom_sub);
    // VectorXd x_init = gtodom_sub->get_x_wrld();
    // std::cout << "x_init: " << x_wrld.size() << std::endl;
    Vector4d q0_vec = x_init.segment(0, 4);
    Vector3d p0 = x_init.segment(4, 3);
    Vector3d w0 = x_init.segment(7, 3);
    Vector3d v0 = x_init.segment(10, 3);
    
    ///////////////////////////////////////////////////////////////////////////////////
    Logger_eMPC = sim_mpc(q0_vec, p0, w0, v0, param, cmd_pub, gtodom_sub);


    std::cout << "MPC finished!" << std::endl;

    cmd_pub->vrx_thrust_left_pub(0.0);
    cmd_pub->vrx_thrust_right_pub(0.0);
    cmd_pub->mavlink_thrust_pub({0.0, 0.0});
    
    // Destroy nodes and shutdown
    rclcpp::shutdown();

    ofstream file;
    if (mode == 1) {
        file.open("zigzag_data.bin", ios::binary);
    } else {
        file.open("circling_data.bin", ios::binary);
    }
    file.write(reinterpret_cast<const char*>(&Logger_eMPC), sizeof(Logger));
    file.close();

    std::cout << "Data saved!" << std::endl;

    // Plotting
    plot_mpc(Logger_eMPC, param);
    std::cout << "Plot finished!" << std::endl;

    return 0;
}