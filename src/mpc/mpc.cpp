#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "include/mpc/plot.h"
#include "include/mpc/param_class.h"
#include "include/mpc/controller/eMPC.h"
#include "include/mpc/utils/LieGroup.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <memory>
#include <filesystem>

using namespace std::chrono_literals;
using namespace Eigen;

class MPC : public rclcpp::Node
{
public:
    MPC() : Node("mpc_node"),
                    i_wp_(0),
                    reached_final_(false),
                    initialized_wp_(true)
    {
        // Load parameters
        std::filesystem::path source_file(__FILE__);
        std::filesystem::path source_dir = source_file.parent_path();
        std::string yaml_path = source_dir.string() + "/config/mpc.yaml";

        param_ = std::make_shared<Param>(yaml_path);


        // Create subscribers
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            param_->path_topic, 10,
            std::bind(&MPC::cbPath, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            param_->pose_topic, 10,
            std::bind(&MPC::cbOdom, this, std::placeholders::_1));

        // Create publishers
        left_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/left/thrust", 10);
        right_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/right/thrust", 10);
        mavlink_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/mavlink/thrust_commands", 10);
        
        // ─────── Timer-Based MPC Loop ───────
        // timer_ = this->create_wall_timer(40ms, std::bind(&MPC::run_mpc, this));  // 25 Hz loop on Yui's

        RCLCPP_INFO(this->get_logger(), "MPC path follower ready ✔");
    }

    // ───────── MPC Loop ─────────
    void run_mpc() {
        if (reached_final_) {
            RCLCPP_INFO(this->get_logger(), "Path already completed");
            return;
        }

        nav_msgs::msg::Odometry odom_copy;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            if (!last_odom_) {
                RCLCPP_INFO(this->get_logger(), "No odometry data received yet");
                return;
            }
            odom_copy = *last_odom_;  // safe copy, unlocks after this scope
        }
        odomTransform(odom_copy);

        // ───────── Initialize Zigzag/Circling waypoints if needed. ─────────
        if (initialized_wp_ && i_wp_ == 0) {
            param_->set_desired_init_position(x_wrld_);
            param_->generate_zigzag_waypoints();
            // param_->generate_circle_waypoints();
            param_->waypoints2twists();
            int Nsim = static_cast<int>(param_->X_ref.rows());
            logger_ = std::make_shared<Logger>(Nsim, param_->Nt);
            initialized_wp_ = false;
        }

        // ───────── Check if there is a valid path ─────────
        if (param_->X_ref.size() == 0) {
            RCLCPP_INFO(this->get_logger(), "No path to follow.");
            return;
        }

        // ───────── Current waypoint from the reference trajectory ─────────
        VectorXd X_ref_rt = param_->X_ref.row(i_wp_).transpose();
        const int Nt = param_->Nt;
        MatrixXd xi_ref_rt = param_->xi_ref.block(i_wp_, 0, Nt, param_->xi_ref.cols());

        // ───────── Check if reached the final waypoint ─────────
        if (i_wp_ >= param_->X_ref.rows() - Nt) {
            reached_final_ = true;
            stopBoat();
            RCLCPP_INFO(this->get_logger(), "Finished path 🎉");
            plot_mpc(*logger_, *param_);
            return;
        }

        auto start = std::chrono::steady_clock::now();

        // ───────── MPC ─────────
        Vector2d u;
        u = eMPC(x_wrld_, X_ref_rt, xi_ref_rt, *param_);

        // ───────── Publish thrust commands ─────────
        std_msgs::msg::Float64 msg_right;
        msg_right.data = u(0);
        right_thrust_pub_->publish(msg_right);

        std_msgs::msg::Float64 msg_left;
        msg_left.data = u(1);
        left_thrust_pub_->publish(msg_left);

        std_msgs::msg::Float64MultiArray mavlink_msg;
        mavlink_msg.data = {u(0), u(1)};
        mavlink_thrust_pub_->publish(mavlink_msg);

        RCLCPP_INFO(this->get_logger(), "right: %7.2f, left: %7.2f", u(0), u(1));

        // ───────── Logging ─────────
        Matrix4d Xd = Matrix4d::Identity();
        Matrix3d rot_ref = quat2rotm(X_ref_rt.segment(0, 4));
        Xd.block<3, 3>(0, 0) = rot_ref;
        Xd.block<3, 1>(0, 3) = X_ref_rt.segment(4, 3);

        Matrix4d X0 = Matrix4d::Identity();
        Matrix3d rot0 = quat2rotm(x_wrld_.segment(0, 4));
        X0.block<3, 3>(0, 0) = rot0;
        X0.block<3, 1>(0, 3) = x_wrld_.segment(4, 3);

        Matrix4d Xerr = (X0.inverse() * Xd).log();
        Matrix4d XXerr = X0.inverse() * Xd;

        VectorXd x00(6);
        x00.segment(0, 3) << Xerr(2, 1), Xerr(0, 2), Xerr(1, 0);
        x00.segment(3, 3) = Xerr.block<3, 1>(0, 3);

        VectorXd err2(4);
        err2 << std::sqrt((XXerr.block<3, 3>(0, 0).log().array().square().sum()) / 2), XXerr.block<3, 1>(0, 3);

        // Save data in Logger (assumed Logger has these vector members)
        logger_->X.block(0, i_wp_, logger_->X.rows(), 1) = x_wrld_;
        logger_->U.block(0, i_wp_, logger_->U.rows(), 1) = u;
        logger_->Err.block(0, i_wp_, logger_->Err.rows(), 1) = x00;
        logger_->Err2.block(0, i_wp_, logger_->Err2.rows(), 1) = err2;

        auto end = std::chrono::steady_clock::now();
        logger_->taken_time += std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
        i_wp_++;
    }

private:
    // Callback for the path subscriber
    void cbPath(const nav_msgs::msg::Path::SharedPtr msg)
    {
        // ───────── Store the entire path as a flat list of XY waypoints. ─────────
        path_.clear();
        for (const auto &pose_stamped : msg->poses) {
            double x = pose_stamped.pose.position.x;
            double y = pose_stamped.pose.position.y;
            path_.emplace_back(x, y);
        }
        int Nsim = static_cast<int>(path_.size());
        if (Nsim < 2) {
            RCLCPP_WARN(this->get_logger(), "Received path with less than 2 waypoints");
            return;
        }

        // ───────── Build reference trajectories ─────────
        param_->X_ref.Zero(Nsim, 7);
        param_->xi_ref.Zero(Nsim, 6);

        for (int i = 0; i < Nsim - 1; i++) {
            Vector2d p1 = path_[i];
            Vector2d p2 = path_[i + 1];
            double dx = p2.x() - p1.x();
            double dy = p2.y() - p1.y();
            double yaw = std::atan2(dy, dx);

            // Create quaternion from yaw (roll and pitch = 0)
            AngleAxisd aa(yaw, Vector3d::UnitZ());
            Quaterniond quat(aa);

            // Each row in X_ref: [qx, qy, qz, qw, px, py, pz]
            VectorXd row(7);
            row << quat.x(), quat.y(), quat.z(), quat.w(), p1.x(), p1.y(), 0.0;

            param_->X_ref.row(i) = row.transpose();
        }

        // Generate twist trajectory from waypoints
        param_->waypoints2twists();

        logger_ = std::make_shared<Logger>(Nsim, param_->Nt);
        i_wp_ = 0;
        reached_final_ = false;
        RCLCPP_INFO(this->get_logger(), "Received path with %d waypoints", Nsim);
    }

    // Callback for the odometry subscriber
    void cbOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        last_odom_ = msg;
    }

    // Stop the boat by publishing zeros to the thruster topics.
    void stopBoat()
    {
        std_msgs::msg::Float64 zero_msg;
        zero_msg.data = 0.0;
        left_thrust_pub_->publish(zero_msg);
        right_thrust_pub_->publish(zero_msg);

        std_msgs::msg::Float64MultiArray mavlink_msg;
        mavlink_msg.data = {0.0, 0.0};
        mavlink_thrust_pub_->publish(mavlink_msg);
        RCLCPP_INFO(this->get_logger(), "Boat stopped");
    }

    // Transform the incoming odometry message to create a world state vector.
    void odomTransform(const nav_msgs::msg::Odometry &msg)
    {
        // Position (world frame)
        double p_x = msg.pose.pose.position.x;
        double p_y = msg.pose.pose.position.y;
        double p_z = msg.pose.pose.position.z;
        Vector3d p_odom(p_x, p_y, p_z);

        // Orientation (world frame)
        double q_x = msg.pose.pose.orientation.x;
        double q_y = msg.pose.pose.orientation.y;
        double q_z = msg.pose.pose.orientation.z;
        double q_w = msg.pose.pose.orientation.w;
        Vector4d q_odom(q_x, q_y, q_z, q_w); // x, y , z, w

        // Linear velocity (body frame)
        double v_x = msg.twist.twist.linear.x;
        double v_y = msg.twist.twist.linear.y;
        double v_z = msg.twist.twist.linear.z;
        Vector3d v_body(v_x, v_y, v_z);

        // Angular velocity (body frame)
        double w_x = msg.twist.twist.angular.x;
        double w_y = msg.twist.twist.angular.y;
        double w_z = msg.twist.twist.angular.z;
        Vector3d w_body(w_x, w_y, w_z);

        // Build x_wrld: [q_wrld, p_wrld, w_body, v_body] (total length 13)
        x_wrld_.segment(0, 4) = q_odom;
        x_wrld_.segment(4, 3) = p_odom;
        x_wrld_.segment(7, 3) = w_body;
        x_wrld_.segment(10, 3) = v_body;
    }

    // Member variables
    std::shared_ptr<Param> param_;
    std::shared_ptr<Logger> logger_;
    std::vector<Vector2d> path_;
    std::shared_ptr<nav_msgs::msg::Odometry> last_odom_ = nullptr;
    int i_wp_;
    bool reached_final_;
    bool initialized_wp_;

    // x_wrld_: state vector concatenating quaternion (4), position (3),
    // angular velocity (3) and linear velocity (3)
    VectorXd x_wrld_ = VectorXd::Zero(13);

    // Subscribers and publishers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_thrust_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_thrust_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mavlink_thrust_pub_;
    std::mutex odom_mutex_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MPC>();
    // rclcpp::spin(node);
    // rclcpp::shutdown();
    
    rclcpp::Rate rate(20);  // ~20 Hz = 50 ms, slightly slower than MPC time

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);  // update latest odometry

        node->run_mpc();          // MPC control loop

        rate.sleep();
    }

    rclcpp::shutdown();
    
    return 0;
}