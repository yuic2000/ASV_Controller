import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32MultiArray
from enum import Enum, auto

import minimalmodbus
import serial
import signal


def create_modbus_device(port: str, slave_id: int):
    dev = minimalmodbus.Instrument(port, slave_id)
    dev.serial.baudrate = 38400
    dev.serial.parity = serial.PARITY_NONE
    dev.serial.stopbits = 1
    dev.serial.bytesize = 8
    dev.serial.timeout = 0.1
    dev.mode = minimalmodbus.MODE_RTU
    return dev

class ESCState(Enum):
    IDLE = 0
    RUN = 1
    FAULT = 2
    RESET = 3

class ESC_log():
    def __init__(self):
        self.ID = 0
        self.name = ""
        self.speed = 0
        self.current = 0
        self.voltage = 0.0
        self.driver_temp = 0
        # self.motor_temp = 0
        self.error_code = 0
        self.online_timer = 5


class DualESCController(Node):
    def __init__(self):
        super().__init__('dual_esc_controller')

        # === Config ===
        self.port = '/dev/ttyUSB0'
        self.use_dual_esc = False  # Set False to debug with right ESC only
        self.right_ID = 1
        self.left_ID = 2 if self.use_dual_esc else None
        
        # === ESC Setup ===
        self.right_esc = create_modbus_device(self.port, self.right_ID)
        self.configure_esc(self.right_esc, "Right")

        if self.use_dual_esc:
            self.left_esc = create_modbus_device(self.port, self.left_ID)
            self.configure_esc(self.left_esc, "Left")

        self.ESCState = ESCState.IDLE
        self.ESCLog_R = ESC_log()
        self.ESCLog_L = ESC_log() if self.use_dual_esc else None
        self.Error_R = False
        self.Error_L = False

        # === Subscription ===
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/screw/thrust_commands',
            self.thrust_callback,
            10
        )

        self.ESC_thrust_pub = self.create_publisher(Int32MultiArray, '/screw/thrust_speed', 10)
        self.ESC_status_pub = self.create_publisher(Int32MultiArray, '/screw/status', 10)

        self.state_machine = self.create_timer(1.0, self.state_machine_loop)
        self.WD_timer = self.create_timer(1.0, self.ESC_WD)           # ESCs watchdog
        self.status_pub = self.create_timer(1.0, self.publish_ESC_status)  # Publish ESC status
        
        self.get_logger().info(
            f"ESC node initialized. Mode: {'Dual ESC' if self.use_dual_esc else 'Right ESC Only'}"
        )

    def thrust_callback(self, msg: Float64MultiArray):
        if self.ESCState != ESCState.RUN:
            return

        try:
            self.publish_thrust(msg)

        except Exception as e:
            self.get_logger().error(f"Modbus write error: {e}")

    def thrust_to_cmd(self, thrust_cmd):
        """Map thrust command to ESC register value."""
        if thrust_cmd == 0 or thrust_cmd > 150 or thrust_cmd < -100:
            return 0        
        elif thrust_cmd > 0:
            return int(thrust_cmd)
        elif thrust_cmd < 0:
            return int(65535 + int(thrust_cmd))

    def configure_esc(self, esc: minimalmodbus.Instrument, name: str):
        try:
            esc.write_register(0x000E, 2, functioncode=6)  # Pn14: Data Channel = MODBUS (2)
            esc.write_register(0x000F, 2, functioncode=6)  # Pn15: Control Channel = MODBUS (2)
            esc.write_register(0x0010, 1, functioncode=6)  # Pn16: Control Mode = Torque (Open Loop) (1)
            ID = esc.read_register(0x003F)                 # Pn13: Modbus ID
            data_chn = esc.read_register(0x000E)           # Pn14: Data Channel = MODBUS (2)
            control_chn = esc.read_register(0x000F)        # Pn15: Control Channel = MODBUS (2)
            mode = esc.read_register(0x0010)               # Pn16: Control Mode = Torque (Open Loop) (1)

            self.get_logger().info(
                f"{name} ESC ID={ID}, Data Channel={data_chn}, Control Channel={control_chn}, Mode={mode}")
        except Exception as e:
            self.get_logger().error(f"{name} ESC configuration failed: {e}")
    
    def read_esc_status(self, esc: minimalmodbus.Instrument, name: str):
        log = ESC_log()
        log.name = name
        try:
            log.online_timer = 5                           # Watchdog
            log.ID = esc.read_register(0x003F)             # Pn13: Modbus ID
            speed = esc.read_register(0x0089)
            log.current = esc.read_register(0x008A)
            voltage_raw = esc.read_register(0x008B)        # Corrected voltage register
            log.error_code = esc.read_register(0x008C)
            driver_temp_raw = esc.read_register(0x0090)
            # motor_temp_raw = esc.read_register(0x0091)
    
            log.speed = speed - 65536 if speed >= 32768 else speed
            log.voltage = voltage_raw * 0.1
            log.driver_temp = driver_temp_raw - 40
    
            self.get_logger().info(
                f"[ESC {log.ID}]: Speed={log.speed}rpm, Current={log.current}A, "
                f"Voltage={log.voltage:.1f}V, Driver Temp={log.driver_temp}°C, "
                f"Error Code=0x{log.error_code:04X}"
            )

        except Exception as e:
            self.get_logger().error(f"Failed to read {log.name} ESC status: {e}")
            log.error_code = 0xFFFF
        
        return log

    def ESC_status_check(self):
        self.ESCLog_R = self.read_esc_status(self.right_esc, "Right")
        if self.use_dual_esc:
            self.ESCLog_L = self.read_esc_status(self.left_esc, "Left")
        
        if self.ESCLog_R.error_code:
            self.Error_R = True
            self.get_logger().error(f"Right ESC Error: 0x{self.ESCLog_R.error_code:04X}")

        if (self.use_dual_esc and self.ESCLog_L.error_code):
            self.Error_L = True
            self.get_logger().error(f"Left ESC Error: 0x{self.ESCLog_L.error_code:04X}")

        if self.ESCLog_R.online_timer == 0:
            self.Error_R = True
            self.get_logger().error(f"Right ESC timeout")
        if (self.use_dual_esc and self.ESCLog_L.online_timer == 0):
            self.Error_L = True
            self.get_logger().error(f"Left ESC timeout")

        return (self.Error_R or self.Error_L)

    def ESC_WD(self):
        if self.ESCLog_R.online_timer != 0:
            self.ESCLog_R.online_timer -= 1
        if self.use_dual_esc and self.ESCLog_L.online_timer != 0:
            self.ESCLog_L.online_timer -= 1

    def state_machine_loop(self):
        self.Error_R, self.Error_L = False, False       # no error
        error = self.ESC_status_check()
        if self.ESCState == ESCState.IDLE:
            self.stop_motor()
            if error:
                self.ESCState = ESCState.FAULT
                print("IDLE 2 FAULT")
            else:
                self.ESCState = ESCState.RUN
                print("IDLE 2 RUN")

        
        elif self.ESCState == ESCState.RUN:
            ## publish thrusts for testing and debugging only
            ## should subscribe to '/screw/thrust_commands'
            # self.publish_thrust(Float64MultiArray(data=[10.0, 0.0]))
            if error:
                self.ESCState = ESCState.FAULT
                print("RUN 2 FAULT")
            else:
                self.ESCState = ESCState.RUN
        
        elif self.ESCState == ESCState.FAULT:
            self.stop_motor()
            if error:
                self.ESCState = ESCState.FAULT
            else:
                self.ESCState = ESCState.RESET
                print("FAULT 2 RESET")

        elif self.ESCState == ESCState.RESET:
            self.stop_motor()
            counter = 5
            if error:
                self.ESCState = ESCState.FAULT
                print("RESET 2 FAULT")
            else:
                counter -= 1
                if counter <= 0:
                    self.ESCState = ESCState.IDLE
                    print("RESET 2 IDLE")

    def stop_motor(self):
        self.publish_thrust(Float64MultiArray(data=[0.0, 0.0]))


    def publish_thrust(self, msg: Float64MultiArray):
        """Publish thrust values for logging."""
        right_thrust = int(msg.data[0])
        left_thrust = int(msg.data[1]) if self.use_dual_esc else 0

        right_val = self.thrust_to_cmd(right_thrust)
        left_val = self.thrust_to_cmd(left_thrust)

        self.right_esc.write_register(0x003A, right_val, functioncode=6)

        if self.use_dual_esc:
            self.left_esc.write_register(0x003A, left_val, functioncode=6)

        self.get_logger().info(
            f"R={right_thrust} N, "
            f"L={'N/A' if not self.use_dual_esc else f'{left_thrust} N'}"
        )
        
        ## Publish to a topic for logging
        pub_msg = Int32MultiArray()
        pub_msg.data = [0] * 4
        pub_msg.data[0] = right_thrust          # newton
        pub_msg.data[1] = left_thrust           # newton
        pub_msg.data[2] = self.ESCLog_R.speed   # rpm
        pub_msg.data[3] = self.ESCLog_L.speed if self.use_dual_esc else 0  # rpm
        self.ESC_thrust_pub.publish(pub_msg)

    def publish_ESC_status(self):
        """Publish motor status for logging."""
        msg = Int32MultiArray()
        msg.data = [0] * 8
        msg.data[0] = int(self.ESCLog_R.voltage * 10)
        msg.data[1] = self.ESCLog_R.current
        msg.data[2] = self.ESCLog_R.driver_temp
        msg.data[3] = self.ESCLog_R.error_code
        msg.data[4] = int(self.ESCLog_L.voltage * 10) if self.use_dual_esc else 0
        msg.data[5] = self.ESCLog_L.current           if self.use_dual_esc else 0
        msg.data[6] = self.ESCLog_L.driver_temp       if self.use_dual_esc else 0
        msg.data[7] = self.ESCLog_L.error_code        if self.use_dual_esc else 0
        self.ESC_status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DualESCController()

    def shutdown_handler(signum, frame):
        node.get_logger().info("Signal received, stopping motors and shutting down...")
        node.stop_motor()
        node.destroy_node()
        rclpy.shutdown()
        exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)  # Ctrl+C
    signal.signal(signal.SIGTSTP, shutdown_handler) # Ctrl+Z

    try:
        rclpy.spin(node)
    finally:
        node.stop_motor()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()