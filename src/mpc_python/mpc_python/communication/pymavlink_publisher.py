import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import time

class PwmCommandPublisher(Node):
    def __init__(self):
        super().__init__('pwm_command_publisher')
        self.get_logger().info('PWM Command Publisher Node Started')

        # ROS 2 Publisher
        self.publisher = self.create_publisher(Int32MultiArray, '/channel_pwm', 10)

        # Timer to publish commands periodically
        self.timer = self.create_timer(2.0, self.publish_pwm)

        # Current PWM signals to publish
        # self.pwm_values = [
        #     [1500, 1500, 1450, 1500, 0, 0, 0, 0]
        # ]
        self.pwm_values = [1500, 1500, 1450, 1500, 0, 0, 0, 0]
        # self.current_index = 0

    def publish_pwm(self):
        """Publish PWM signals."""
        msg = Int32MultiArray()
        # msg.data = self.pwm_values[self.current_index]
        msg.data = self.pwm_values
        self.publisher.publish(msg)
        self.get_logger().info(f'Published PWM: {msg.data}')

        # Cycle through different PWM values
        # self.current_index = (self.current_index + 1) % len(self.pwm_values)


def main(args=None):
    rclpy.init(args=args)
    node = PwmCommandPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()


