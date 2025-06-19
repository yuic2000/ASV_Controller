#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
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
                    initialized_wp_(false)
    {
        // Load parameters
        std::filesystem::path source_file(__FILE__);
        std::filesystem::path source_dir = source_file.parent_path();
        std::string yaml_path = source_dir.string() + "/config/mpc.yaml";

        param_ = std::make_shared<Param>(yaml_path);
        MPC_Hz = param_->Hz;

        // Create subscribers
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            param_->path_topic, 10,
            std::bind(&MPC::cbPath, this, std::placeholders::_1));

        // odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(     // if using "/wamv/sensors/position/ground_truth_odometry"
        //     param_->odom_topic, 10,
        //     std::bind(&MPC::cbOdom, this, std::placeholders::_1));
        
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(     // if using "/gt_pose"
            param_->pose_topic, 10,
            std::bind(&MPC::cbPose, this, std::placeholders::_1));

        twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(    // if using "/wamv/inekf_odom_correction/twist"
            param_->twist_topic, 10,
            std::bind(&MPC::cbTwist, this, std::placeholders::_1));
        
        // twist_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        //     param_->twist_topic, 10,
        //     std::bind(&MPC::cbTwist, this, std::placeholders::_1));

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&MPC::cbGoal, this, std::placeholders::_1));

        // Create publishers
        left_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/left/thrust", 10);
        right_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/right/thrust", 10);
        mavlink_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/mavlink/thrust_commands", 10);
        
        // ─────── Timer-Based MPC Loop ───────
        // timer_ = this->create_wall_timer(40ms, std::bind(&MPC::run_mpc, this));  // 25 Hz loop on Yui's

        RCLCPP_INFO(this->get_logger(), "MPC path follower ready ✔");
    }

    double MPC_Hz;  // MPC frequency

    // ───────── MPC Loop ─────────
    void run_mpc() {
        if (reached_final_) {
            RCLCPP_INFO(this->get_logger(), "Path already completed, Awaiting for new path.");
            return;
        }
        // ───────── 1. if using "/gt_pose" and "/wamv/inekf_odom_correction/twist" ─────────
        geometry_msgs::msg::PoseStamped pose_copy;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            if (!last_pose_) {
                RCLCPP_INFO(this->get_logger(), "No Pose data received yet");
                return;
            }
            pose_copy = *last_pose_;  // safe copy, unlocks after this scope
        }
        geometry_msgs::msg::TwistStamped twist_copy;
        {
            std::lock_guard<std::mutex> lock(twist_mutex_);
            if (!last_twist_) {
                RCLCPP_INFO(this->get_logger(), "No Twist data received yet");
                return;
            }
            twist_copy = *last_twist_;  // safe copy, unlocks after this scope
        }
        PoseTwistTransform(pose_copy, twist_copy);

        // ───────── 2. if using "/wamv/sensors/position/ground_truth_odometry" ─────────
        // nav_msgs::msg::Odometry odom_copy;
        // {
        //     std::lock_guard<std::mutex> lock(odom_mutex_);
        //     if (!last_odom_) {
        //         RCLCPP_INFO(this->get_logger(), "No odometry data received yet");
        //         return;
        //     }
        //     odom_copy = *last_odom_;  // safe copy, unlocks after this scope
        // }
        // odomTransform(odom_copy);

        // ───────── Initialize Zigzag/Circling waypoints if needed. ─────────
        if (initialized_wp_ && i_wp_ == 0) {
            param_->set_desired_init_position(x_wrld_);
            param_->generate_zigzag_waypoints();
            // param_->generate_circle_waypoints();
            param_->waypoints2twists();
            Nsim = static_cast<int>(param_->X_ref.rows());
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
        
        Matrix4d goalErr_mat = X0.inverse() * Xd_goal;
        double goalErr = std::sqrt(std::pow(goalErr_mat(0, 3), 2) + std::pow(goalErr_mat(1, 3), 2) + std::pow(goalErr_mat(2, 3), 2));
        RCLCPP_INFO(this->get_logger(), "Goal error: %f", goalErr);
 
        // if(goalErr < param_->goal_tol || i_wp_ >= param_->X_ref.rows() - Nt) {
        //     RCLCPP_INFO(this->get_logger(), "Goal reached! Stopping the boat.");
        //     stopBoat();
        //     reached_final_ = true;
        //     plot_mpc(*logger_, *param_);
        //     return;
        // }
        
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

private:
    // Callback for the path subscriber
    void cbPath(const nav_msgs::msg::Path::SharedPtr msg)
    {
        // ───────── Store the entire path as a flat list of XY waypoints. ─────────
        original_path_.clear();
        dense_path_.clear();
        for (const auto &pose_stamped : msg->poses) {
            double x = pose_stamped.pose.position.x;
            double y = pose_stamped.pose.position.y;
            original_path_.emplace_back(x, y);
        }

        // Process Path to Dense Waypoints (fixed step)
        PathProcessing();
        RCLCPP_INFO(this->get_logger(), "Received path with %d waypoints", PathSize);

        if (PathSize < 2) {
            RCLCPP_WARN(this->get_logger(), "Received path with less than 2 waypoints");
            return;
        }

        // ───────── Build reference trajectories ─────────
        Nsim = PathSize + param_->Nt;  // Total number of simulation steps
        param_->X_ref = MatrixXd::Zero(Nsim, 7);
        double yaw = 0.0;
        for (int i = 0; i < PathSize; i++) {
            Vector2d p1 = dense_path_[i];
        
            if (i < PathSize - 1) {     // yaw at last point remains the same
                Vector2d p2 = dense_path_[i + 1];
                double dx = p2(0) - p1(0);
                double dy = p2(1) - p1(1);
                yaw = std::atan2(dy, dx);
            }
        
            AngleAxisd aa(yaw, Vector3d::UnitZ());
            Quaterniond quat(aa);
        
            VectorXd new_state(7);
            new_state << quat.x(), quat.y(), quat.z(), quat.w(), p1(0), p1(1), 0.0;        
            param_->X_ref.row(i) = new_state.transpose();
        }
        for (int i = PathSize; i < Nsim; i++) {
            param_->X_ref.row(i) = param_->X_ref.row(PathSize - 1);
        }

        // Generate twist trajectory from waypoints
        param_->waypoints2twists();

        logger_ = std::make_shared<Logger>(Nsim, param_->Nt);

        i_wp_ = 0;
        reached_final_ = false;
        RCLCPP_INFO(this->get_logger(), "Set %d waypoints for MPC", PathSize);
        RCLCPP_INFO(this->get_logger(), "Set %d simulation steps", Nsim);
        RCLCPP_INFO(this->get_logger(), "Set X_ref with %zu states", param_->X_ref.rows());
        RCLCPP_INFO(this->get_logger(), "Set xi_ref with %zu states", param_->xi_ref.rows());
    }

    void cbGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Position (world frame)
        double p_x = msg->pose.position.x;
        double p_y = msg->pose.position.y;
        // double p_z = msg->pose.position.z;
        double p_z = 0.0;
        Vector3d p_odom(p_x, p_y, p_z);

        // Orientation (world frame)
        double q_x = msg->pose.orientation.x;
        double q_y = msg->pose.orientation.y;
        double q_z = msg->pose.orientation.z;
        double q_w = msg->pose.orientation.w;
        Vector4d q_odom(q_x, q_y, q_z, q_w); // x, y , z, w

        x_goal.segment(0, 4) = q_odom;  // Set orientation
        x_goal.segment(4, 3) = p_odom;  // Set position

        Matrix3d rot_goal = quat2rotm(x_goal.segment(0, 4));
        Xd_goal.block<3, 3>(0, 0) = rot_goal;
        Xd_goal.block<3, 1>(0, 3) = x_goal.segment(4, 3);
    }

    // Callback for the odometry subscriber
    void cbOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        last_odom_ = msg;
    }

    // Callback for the pose subscriber
    void cbPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        last_pose_ = msg;
    }

    // Callback for the twist subscriber
    void cbTwist(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(twist_mutex_);
        last_twist_ = msg;
    }

    // Callback for the odometry subscriber
    // void cbTwist(const nav_msgs::msg::Odometry::SharedPtr msg)
    // {
    //     auto twist_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
    //     twist_msg->header = msg->header;
    //     twist_msg->twist = msg->twist.twist;
    //     {
    //         std::lock_guard<std::mutex> lock(twist_mutex_);
    //         last_twist_ = twist_msg;
    //     }
    // }

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

    void PoseTwistTransform(const geometry_msgs::msg::PoseStamped& pose_msg, const geometry_msgs::msg::TwistStamped& twist_msg)
    {
        // Position (world frame)
        double p_x = pose_msg.pose.position.x;
        double p_y = pose_msg.pose.position.y;
        // double p_z = pose_msg.pose.position.z;
        double p_z = 0.0;
        Vector3d p_odom(p_x, p_y, p_z);

        // Orientation (world frame)
        double q_x = pose_msg.pose.orientation.x;
        double q_y = pose_msg.pose.orientation.y;
        double q_z = pose_msg.pose.orientation.z;
        double q_w = pose_msg.pose.orientation.w;
        Vector4d q_odom(q_x, q_y, q_z, q_w); // x, y , z, w

        double yaw = std::atan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y * q_y + q_z * q_z));

        // Linear velocity (body frame)
        double v_x = twist_msg.twist.linear.x * std::cos(yaw) + twist_msg.twist.linear.y * std::sin(yaw);
        double v_y = twist_msg.twist.linear.y * std::cos(yaw) - twist_msg.twist.linear.x * std::sin(yaw);
        // double v_x = twist_msg.twist.linear.x;
        // double v_y = twist_msg.twist.linear.y;
        // double v_z = twist_msg.twist.linear.z;
        double v_z = 0.0;
        Vector3d v_body(v_x, v_y, v_z);

        // Angular velocity (body frame)
        double w_x = 0.0;
        double w_y = 0.0;
        // double w_x = twist_msg.twist.angular.x;
        // double w_y = twist_msg.twist.angular.y;
        double w_z = twist_msg.twist.angular.z;
        Vector3d w_body(w_x, w_y, w_z);

        // Build x_wrld: [q_wrld, p_wrld, w_body, v_body] (total length 13)
        x_wrld_.segment(0, 4) = q_odom;
        x_wrld_.segment(4, 3) = p_odom;
        x_wrld_.segment(7, 3) = w_body;
        x_wrld_.segment(10, 3) = v_body;
    }

    // Process Path to Dense Waypoints (fixed step), interpolating and stacking last points
    void PathProcessing()
    {
        if (original_path_.size() < 2) {
            dense_path_ = original_path_;
        }

        // Create an array of distances between consecutive points
        std::vector<double> distances;
        for (size_t i = 0; i < original_path_.size() - 1; ++i) {
            distances.push_back((original_path_[i + 1] - original_path_[i]).norm());
        }
        double total_distance = std::accumulate(distances.begin(), distances.end(), 0.0);

        // Define the number of interpolation points
        int total_sim = static_cast<int>(
                std::ceil((total_distance / param_->v_surge) / param_->dt)
            );
        for (size_t i = 0; i < original_path_.size() - 1; ++i) {
            const auto& p1 = original_path_[i];
            const auto& p2 = original_path_[i + 1];
            double segment_length = distances[i];
            int num_points = static_cast<int>(std::ceil(segment_length / total_distance * total_sim));

            // Interpolate points along the segment
            for (int j = 0; j < num_points; ++j) {
                double t = static_cast<double>(j) / num_points;
                Vector2d interpolated_point = p1 + t * (p2 - p1);
                dense_path_.push_back(interpolated_point);
            }
        }
        PathSize = static_cast<int>(dense_path_.size());
    }


    // Member variables
    std::shared_ptr<Param> param_;
    std::shared_ptr<Logger> logger_;
    std::vector<Vector2d> original_path_;
    std::vector<Vector2d> dense_path_;
    std::shared_ptr<nav_msgs::msg::Odometry> last_odom_ = nullptr;
    std::shared_ptr<geometry_msgs::msg::PoseStamped> last_pose_ = nullptr;
    std::shared_ptr<geometry_msgs::msg::TwistStamped> last_twist_ = nullptr;


    int Nsim;               // Number of simulation steps (size of the path), PathSize + Nt
    int PathSize;           // Number of waypoints in the path
    int i_wp_;
    bool reached_final_;
    bool initialized_wp_;

    // x_wrld_: state vector concatenating quaternion (4), position (3),
    // angular velocity (3) and linear velocity (3)
    VectorXd x_wrld_ = VectorXd::Zero(13);
    VectorXd x_goal = VectorXd::Zero(13);  // goal state vector
    Matrix4d Xd_goal = Matrix4d::Identity();  // goal state in world frame

    // Subscribers and publishers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr twist_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_thrust_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_thrust_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mavlink_thrust_pub_;
    std::mutex odom_mutex_;
    std::mutex pose_mutex_;
    std::mutex twist_mutex_;
    rclcpp::TimerBase::SharedPtr timer_;
};

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

int main(int argc, char **argv)
{
    Eigen::setNbThreads(10); // Use 10 threads

    rclcpp::init(argc, argv);
    auto node = std::make_shared<MPC>();
    
    rclcpp::Rate rate(node->MPC_Hz);  // ~20 Hz = 50 ms, slightly slower than MPC time

    ctrl_c_pressed = false;
    signal(SIGINT, ctrlc);
    signal(SIGTERM, ctrlc);

    while (rclcpp::ok() && !ctrl_c_pressed) {
        rclcpp::spin_some(node);  // update latest odometry
        node->run_mpc();          // MPC control loop
        rate.sleep();
    }


    node->stopBoat();  // Stop the boat when exiting
    rclcpp::shutdown();
    
    return 0;
}