#include <random>
#include <fstream>
#include <filesystem>
#include <omp.h>


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
    
    // Eigen::setNbThreads(6); // Use 6 threads
    omp_set_num_threads(8); // Control OpenMP thread count

    std::filesystem::path source_file(__FILE__);
    std::filesystem::path source_dir = source_file.parent_path();
    std::string yaml_path = source_dir.string() + "/config/station_keeping.yaml";

    Param param(yaml_path);
    int mode = param.mode;              // 1: zigzag, 2: circling
    int Nsim = param.Nsim;              // Total simulation time + Planning time
    int Nt = param.Nt;                  // MPC planning time


    // Initialize MPC node
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

    // // Set desired starting position
    // x_init = VectorXd::Zero(13);                 // when using boat model, start at (0, 0, 0)
    param.set_desired_init_position(x_init);

    param.StationKeeping();                         // set reference trajectory for station keeping


    Logger Logger_eMPC(Nsim, Nt);

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