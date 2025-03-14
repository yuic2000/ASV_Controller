#include "../../include/mpc/communication/gtodom_sub.hpp"

// using namespace std::chrono_literals;

GtOdomSubscriberNode::GtOdomSubscriberNode() : Node("ground_truth_odometry_subscriber") 
{
    odom_topic = "/wamv/sensors/position/ground_truth_odometry";
    RCLCPP_INFO(this->get_logger(), "Subscribe to odometry topic: %s", odom_topic.c_str());
    // RCLCPP_INFO(this->get_logger(), "Ground truth odometry subscriber node created");

    // translation_odom_src_to_wrld << -532.64, 162.5, -0.07;
    translation_odom_src_to_wrld << 0, 0, 0;
    quat_odom_src_to_rot_wrld = Eigen::Quaterniond(1, 0, 0, 0);    // quaternion, w, x, y, z

    odom_pos_src2wrld.block<3, 3>(0, 0) = quat_odom_src_to_rot_wrld.toRotationMatrix();
    odom_pos_src2wrld.block<3, 1>(0, 3) = translation_odom_src_to_wrld;
    odom_pos_src2wrld.block<1, 3>(3, 0) = Eigen::RowVector3d::Zero();
    odom_pos_src2wrld(3, 3) = 1.0;


    gtodom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 50, std::bind(&GtOdomSubscriberNode::odom_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Ground truth odometry subscriber node created");
}

void GtOdomSubscriberNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    Eigen::Vector3d p_odom(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Quaterniond q_odom(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Vector3d v_body(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    Eigen::Vector3d w_body(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);

    // Map frame
    Eigen::Matrix4d odomsrc_coord;
    odomsrc_coord.block<3, 3>(0, 0) = q_odom.normalized().toRotationMatrix();
    odomsrc_coord.block<3, 1>(0, 3) = p_odom;
    odomsrc_coord.block<1, 3>(3, 0) = Eigen::RowVector3d::Zero();
    odomsrc_coord(3, 3) = 1.0;

    Eigen::Matrix4d wrld_coord = odom_pos_src2wrld.inverse() * odomsrc_coord;
    Eigen::Quaterniond q_wrld_quat(wrld_coord.block<3, 3>(0, 0));       // w, x, y, z, quaternion type
    Eigen::Vector4d q_wrld = q_wrld_quat.coeffs();                      // x, y, z, w, vector type
    Eigen::Vector3d p_wrld = wrld_coord.block<3, 1>(0, 3);

    x_wrld.resize(13);
    x_wrld << q_wrld, p_wrld, w_body, v_body;
}

Eigen::VectorXd GtOdomSubscriberNode::get_x_wrld() const {
    return x_wrld;
}


// int main(int argc, char *argv[]) {
//     rclcpp::init(argc, argv);
//     auto Odom_sub = std::make_shared<GtOdomSubscriberNode>();

//     rclcpp::spin(Odom_sub);
//     rclcpp::shutdown();
//     return 0;
// }