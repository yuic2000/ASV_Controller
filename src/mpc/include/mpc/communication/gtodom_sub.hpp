#pragma once

#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

class GtOdomSubscriberNode : public rclcpp::Node 
{
public:
    GtOdomSubscriberNode();

    Eigen::VectorXd get_x_wrld() const;

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gtodom_sub;
    
    std::string odom_topic;
    Eigen::Vector3d translation_odom_src_to_wrld;
    Eigen::Quaterniond quat_odom_src_to_rot_wrld;
    Eigen::Matrix4d odom_pos_src2wrld;
    Eigen::VectorXd x_wrld;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

};