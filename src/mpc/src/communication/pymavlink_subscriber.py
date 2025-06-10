import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from pymavlink import mavutil
import time

class PropellerController(Node):
    def __init__(self):
        super().__init__('propeller_controller')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/channel_pwm',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # MAVLink connection setup
        self.master = mavutil.mavlink_connection('udpin:192.168.2.1:14550')
        self.master.wait_heartbeat()
        self.get_logger().info('Connected to MAVLink')

        # Arm the vehicle
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True)
        self.get_logger().info(f'Vehicle armed,Arming result:{ack.result}')

        # Set manual mode
        mode = 'MANUAL'
        if mode not in self.master.mode_mapping():
            self.get_logger().error(f'Unknown mode: {mode}')
            self.get_logger().info(f'Available modes: {list(self.master.mode_mapping().keys())}')
            rclpy.shutdown()
            return

        mode_id = self.master.mode_mapping()[mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        self.get_logger().info(f'Mode set to {mode}')

        # Initialize RC channel values
        self.rc_channel_values = [1500, 1500, 1500, 1500, 0, 0, 0, 0]
        self.running = True

        # Start loop to continuously send PWM signals
        self.create_timer(0.1, self.send_rc_commands)

    def set_rc_channel_pwm(self, channel_id, pwm):
        """Set RC channel PWM value."""
        if 1 <= channel_id <= 18:
            self.rc_channel_values[channel_id - 1] = pwm
        else:
            self.get_logger().warning(f'Channel {channel_id} does not exist.')

    def send_rc_commands(self):
        """Continuously send RC channel override messages."""
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *self.rc_channel_values
        )

    def listener_callback(self, msg):
        """Process incoming PWM commands."""
        if len(msg.data) > 8:
            self.get_logger().warning('Only 8 channels are supported.')
            return

        # Update RC channel values with received data
        for i, pwm in enumerate(msg.data):
            self.set_rc_channel_pwm(i + 1, pwm)
        self.get_logger().info(f'Updated PWM values: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    controller = PropellerController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()