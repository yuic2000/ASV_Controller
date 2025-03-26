import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import numpy as np

class PathSubscriber(Node):
    def __init__(self):
        super().__init__('path_subscriber')
        self.path_topic = '/planned_path'
        self.get_logger().info(f"Subscribing to path topic: {self.path_topic}")

        self.subscription = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            10)

        self.X_ref = np.zeros(7)

    def path_callback(self, msg):
        for i, pose_stamped in enumerate(msg.poses):
            position = pose_stamped.pose.position       # x, y, z
            quaternion = pose_stamped.pose.orientation  # qx, qy, qz, qw
            
            pose = np.array([position.x, position.y, position.z])
            quat = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
            
            if i == 0:  # First entry
                self.X_ref = np.concatenate((pose, quat))
            else:
                self.X_ref = np.vstack((self.X_ref, np.concatenate((pose, quat))))
        
        self.get_logger().info(f"Updated X_ref shape: {self.X_ref.shape}")
        
    def get_X_ref(self):
        return self.X_ref

def main(args=None):
    rclpy.init(args=args)
    node = PathSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()