#include "../../include/mpc/communication/path_sub.hpp"


PathSubscriber::PathSubscriber() : Node("path_subscriber") {
    path_topic_ = "/planned_path";
    RCLCPP_INFO(this->get_logger(), "Subscribing to path topic: %s", path_topic_.c_str());

    subscription_ = this->create_subscription<nav_msgs::msg::Path>(
        path_topic_, 10,
        std::bind(&PathSubscriber::path_callback, this, std::placeholders::_1));
}


const Eigen::MatrixXd& PathSubscriber::get_X_ref() const {
    return X_ref_;
}


void PathSubscriber::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    const int num_poses = msg->poses.size();
    
    if (num_poses > 0) {
        X_ref_.resize(num_poses, 7); // Resize matrix to fit all poses
        
        for (int i = 0; i < num_poses; ++i) {
            const auto& pose_stamped = msg->poses[i];
            const auto& position = pose_stamped.pose.position;
            const auto& quaternion = pose_stamped.pose.orientation;
            
            // Potentially swap order to match your desired format
            X_ref_.row(i) << 
                quaternion.x, quaternion.y, quaternion.z, quaternion.w,
                position.x, position.y, position.z;
        }
        
        RCLCPP_INFO(this->get_logger(), "Updated X_ref size: %ld x %ld", 
                    X_ref_.rows(), X_ref_.cols());
    } 
    
    else {
        RCLCPP_WARN(this->get_logger(), "Received empty path message");
        X_ref_.resize(0, 7);
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
