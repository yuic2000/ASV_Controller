#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class MPCCmdPublishNode : public rclcpp::Node
{
public:
    MPCCmdPublishNode();

    void vrx_thrust_left_pub(double thrust);
    void vrx_thrust_right_pub(double thrust);
    void mavlink_thrust_pub(const std::vector<double>& thrust);

private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vrx_thrust_pub_l_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vrx_thrust_pub_r_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mavlink_thrust_pub_;
};