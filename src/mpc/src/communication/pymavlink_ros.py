import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32MultiArray
from pymavlink import mavutil
import time

class IntegratedMavlinkController(Node):
    def __init__(self):
        super().__init__('mavlink_thrust_controller')
        
        # Initialize MAVLink connection
        self._init_mavlink()
        
        # Initialize PWM values
        self.rc_channel_values = [1500, 1500, 1500, 1500, 0, 0, 0, 0]
        
        # Create a subscriber to listen for thrust commands
        self.subscription = self.create_subscription(
            Float64MultiArray,  # Use a message containing two float values
            '/mavlink/thrust_commands',
            self.thrust_callback,
            10
        )
        # PWM Publisher for logging
        self.Mavlink_pub = self.create_publisher(Int32MultiArray, '/mavlink/thrust_PWM', 10)
        
        # Create a timer to send RC commands periodically (100ms interval)
        # self.create_timer(0.05, self.send_rc_commands)

    def _init_mavlink(self):
        """Initialize MAVLink connection."""
        self.master = mavutil.mavlink_connection('udpin:192.168.2.1:14550')
        self.master.wait_heartbeat()
        self.get_logger().info('MAVLink connection established')
        
        # Arm the vehicle
        self._arm_vehicle()
        
        # Set the flight mode to MANUAL
        self._set_flight_mode('MANUAL')

    def _arm_vehicle(self):
        """Arm the vehicle."""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True)
        self.get_logger().info(f'Vehicle arming status: {ack.result}')

    def _set_flight_mode(self, mode):
        """Set the flight mode."""
        if mode not in self.master.mode_mapping():
            self.get_logger().error(f'Invalid mode: {mode}')
            return
            
        mode_id = self.master.mode_mapping()[mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        self.get_logger().info(f'Flight mode set to: {mode}')

    def thrust_callback(self, msg):
        """Callback to handle thrust commands."""
        start = time.time()
        if len(msg.data) != 2:
            self.get_logger().error('Invalid command format, expected [right_thrust, left_thrust]')
            return
            
        self.right_thrust = msg.data[0] / 9.81      # Newton to kgF
        self.left_thrust = msg.data[1] / 9.81       # Newton to kgF
        
        # Convert thrust to PWM
        self.left_pwm = self._thrust_to_pwm(self.left_thrust)
        self.right_pwm = self._thrust_to_pwm(self.right_thrust)
        
        # Calculate throttle and steering
        self.throttle, self.steering = self._calculate_controls()
        
        # Update RC channel values
        self._update_channels(self.steering, self.throttle)
        self.send_rc_commands()             # Send RC commands to the vehicle
        self.publish_pwm()                  # publish PWM values for logging
        
        self.get_logger().info(
            f'Thrust commands: left={self.left_thrust:.2f}, right={self.right_thrust:.2f} (kgF)\n'
            f'PWM: left={self.left_pwm}, right={self.right_pwm}\n'
            f'Channels: throttle={self.throttle}, steering={self.steering}\n'
        )
        self.get_logger().info(f'Callback execution time: {time.time() - start:.4f} seconds')

    def _thrust_to_pwm(self, thrust):
        """Convert thrust force to PWM using calibration data."""
        calibration = [
            (1100, -2.81), (1110, -2.79), (1136, -2.64), (1162, -2.42),
            (1188, -2.21), (1214, -2.04), (1240, -1.83), (1266, -1.57),
            (1292, -1.42), (1318, -1.20), (1344, -0.98), (1370, -0.82),
            (1396, -0.60), (1422, -0.41), (1448, -0.24), (1474, -0.09),
            (1500, 0), (1526, 0.21), (1552, 0.5), (1578, 0.82),
            (1604, 1.17), (1630, 1.58), (1656, 1.93), (1682, 2.37),
            (1708, 2.76), (1734, 3.23), (1760, 3.57), (1786, 3.99),
            (1812, 4.36), (1838, 4.84), (1864, 5.22), (1890, 5.45),
            (1900, 5.63)
        ]
        
        # Boundary checks
        if thrust <= -2.81: return 1100
        if thrust >= 5.63: return 1900
        
        # Linear interpolation
        for i in range(len(calibration)-1):
            if calibration[i][1] <= thrust <= calibration[i+1][1]:
                x0, y0 = calibration[i]
                x1, y1 = calibration[i+1]
                return int(x0 + (thrust - y0) * (x1 - x0) / (y1 - y0))
        return 1500

    def _calculate_controls(self):
        """Calculate throttle and steering from PWM values."""
        throttle = (self.left_pwm + self.right_pwm) // 2
        steering = 1500 - (self.right_pwm - self.left_pwm) // 2
        return throttle, steering

    def _update_channels(self, steering, throttle):
        """Update RC channel values."""
        # Channel 1: Steering, Channel 3: Throttle
        self.rc_channel_values[0] = steering
        self.rc_channel_values[2] = throttle
        # self.rc_channel_values[0] = 1500
        # self.rc_channel_values[2] = 1500

    def send_rc_commands(self):
        """Send RC override commands periodically."""
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *self.rc_channel_values
        )

    def publish_pwm(self):
        """Publish PWM signals."""
        msg = Int32MultiArray()
        msg.data = [0] * 6
        msg.data[0] = int(self.right_thrust * 1000)     # gF
        msg.data[1] = int(self.left_thrust * 1000)      # gF
        msg.data[2] = self.right_pwm
        msg.data[3] = self.left_pwm
        msg.data[4] = self.throttle
        msg.data[5] = self.steering
        self.Mavlink_pub.publish(msg)

    def stop_motors(self):
        """Safely stop all motors."""
        try:
            self._update_channels(1500, 1500)  # Reset to neutral
            self.send_rc_commands()
            self.publish_pwm()                  # publish PWM values for logging
            self.get_logger().info('Motors stopped successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to stop motors: {str(e)}')

    def __del__(self):
        """Stop motors when the node is destroyed."""
        self._update_channels(1500, 1500)
        self.send_rc_commands()
        self.publish_pwm()                  # publish PWM values for logging
        self.get_logger().info('Motors safely stopped')

def main(args=None):
    rclpy.init(args=args)
    controller = IntegratedMavlinkController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutdown signal received')
    finally:
        controller.stop_motors()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()