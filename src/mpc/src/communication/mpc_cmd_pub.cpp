#include "../../include/mpc/communication/mpc_cmd_pub.hpp"


MPCCmdPublishNode::MPCCmdPublishNode() : Node("mpc_thrust_cmd_publisher")
{
    RCLCPP_INFO(this->get_logger(), "mpc_node is running");

    vrx_thrust_pub_l_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/left/thrust", 10);
    vrx_thrust_pub_r_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/right/thrust", 10);
    mavlink_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/mavlink/thrust_commands", 10);
}

void MPCCmdPublishNode::vrx_thrust_left_pub(double thrust)
{
    auto msg = std_msgs::msg::Float64();
    msg.data = thrust;
    vrx_thrust_pub_l_->publish(msg);
    // RCLCPP_INFO(this->get_logger(), "Publishing left propeller thrust: '%.2f'", msg.data);
}

void MPCCmdPublishNode::vrx_thrust_right_pub(double thrust)
{
    auto msg = std_msgs::msg::Float64();
    msg.data = thrust;
    vrx_thrust_pub_r_->publish(msg);
    // RCLCPP_INFO(this->get_logger(), "Publishing right propeller thrust: '%.2f'", msg.data);
}

void MPCCmdPublishNode::mavlink_thrust_pub(const std::vector<double>& thrust)
{
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = thrust;
    mavlink_thrust_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "right: %.2f, left: %.2f", msg.data[0], msg.data[1]);
}

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);

//     auto mpc_node = std::make_shared<MPCCmdPublishNode>();

//     rclcpp::spin(mpc_node);

//     rclcpp::shutdown();
//     return 0;
// }