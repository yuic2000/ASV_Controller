import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import numpy as np
import matplotlib.pyplot as plt
from math import ceil

class PathSubscriber(Node):
    def __init__(self):
        super().__init__('path_subscriber')
        self.path_topic = '/planning/path'
        self.get_logger().info(f"Subscribing to path topic: {self.path_topic}")

        self.subscription = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            10)

        self.X_ref = np.zeros(7)

    # ───────── callbacks ─────────
    def path_callback(self, msg: Path):
        """Store the entire path as a flat list of XY waypoints and plot the trajectory."""
        self.orig_path = [np.array([p.pose.position.x,
                                    p.pose.position.y]) for p in msg.poses]
        


        # Extract X and Y coordinates
        x_coords_org = [point[0] for point in self.orig_path]
        y_coords_org = [point[1] for point in self.orig_path]
        self.get_logger().info(f"Received original path with {len(self.orig_path)} waypoints.")

        self.interpolatePath()
        x_coords_dense = [point[0] for point in self.interpolated_path]
        y_coords_dense = [point[1] for point in self.interpolated_path]
        self.get_logger().info(f"Interpolated path with {len(self.interpolated_path)} waypoints.")
        # Plot the trajectory
        plt.figure(1)
        plt.plot(x_coords_org, y_coords_org, marker='o', label='Planned Path')
        plt.plot(x_coords_dense, y_coords_dense, marker='.', label='Planned Path')

        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.title('Planned Path Trajectory')
        plt.legend()
        plt.grid()

        self.get_yaw_angle_inWrld()
        self.timestamp = np.linspace(0, len(self.yaw_list) * 0.04, len(self.yaw_list))
        plt.figure(2)
        plt.plot(self.timestamp, self.yaw_list, marker='.', label='Yaw Angle')
        plt.xlabel('Time (s)')
        plt.ylabel('Yaw Angle (radians)')
        plt.title('Yaw Angle Over Time')
        plt.grid()
        plt.show()

    def interpolatePath(self):
        """Interpolate the path to create a smoother trajectory."""
        if len(self.orig_path) < 2:
            self.get_logger().warn("Not enough points to interpolate.")
            return
        
        # Create an array of distances between consecutive points
        distances = [np.linalg.norm(self.orig_path[i] - self.orig_path[i + 1]) for i in range(len(self.orig_path) - 1)]
        total_distance = sum(distances)
        self.get_logger().info(f"Total distance of the path: {total_distance:.2f} meters")
        
        # Define the number of interpolation points
        Nsim = ceil((total_distance / 0.5 + 5) / 0.04)
        self.interpolated_path = []
        
        for i in range(len(self.orig_path) - 1):
            p1 = self.orig_path[i]
            p2 = self.orig_path[i + 1]
            segment_length = distances[i]
            num_points = ceil(segment_length / total_distance * Nsim)
            
            # Interpolate points along the segment
            for j in range(num_points):
                t = j / num_points
                interpolated_point = p1 + t * (p2 - p1)
                self.interpolated_path.append(interpolated_point)

    def get_yaw_angle_inWrld(self):
        """Calculate the yaw angle in the world frame."""
        if len(self.interpolated_path) < 2:
            self.get_logger().warn("Not enough points to calculate yaw angle.")
            return None
        self.yaw_list = []

        for i in range(len(self.interpolated_path) - 1):
            # Calculate the yaw angle between consecutive points
            p1 = self.interpolated_path[i]
            p2 = self.interpolated_path[i+1]
            dx, dy = p2[0] - p1[0], p2[1] - p1[1]
            yaw = np.arctan2(dy, dx)
            self.yaw_list.append(yaw)
        

    # def path_callback(self, msg):
    #     for i, pose_stamped in enumerate(msg.poses):
    #         position = pose_stamped.pose.position       # x, y, z
    #         quaternion = pose_stamped.pose.orientation  # qx, qy, qz, qw
            
    #         pose = np.array([position.x, position.y, position.z])
    #         quat = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
            
    #         if i == 0:  # First entry
    #             self.X_ref = np.concatenate((pose, quat))
    #         else:
    #             self.X_ref = np.vstack((self.X_ref, np.concatenate((pose, quat))))
        
    #     self.get_logger().info(f"Updated X_ref shape: {self.X_ref.shape}")
        
    # def get_X_ref(self):
    #     return self.X_ref

def main(args=None):
    rclpy.init(args=args)
    node = PathSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()