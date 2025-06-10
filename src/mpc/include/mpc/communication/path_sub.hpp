#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"

#include <Eigen/Dense>
#include <array>

class PathSubscriber : public rclcpp::Node {
public:
    PathSubscriber();
    const Eigen::MatrixXd& get_X_ref() const;

private:
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    std::string path_topic_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_;
    Eigen::MatrixXd X_ref_;
};