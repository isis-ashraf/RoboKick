import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial


class SerialBridgeNode(Node):

    def __init__(self):
        super().__init__('serial_bridge')

        self.port_name = '/dev/ttyUSB0'
        self.baud_rate = 115200
        self.ser = None

        self.connect_serial()

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/joint_angles',
            self.angles_callback,
            1
        )
        
        self.create_timer(0.1, self.read_serial)  # check for incoming serial data every 100 ms
        self.get_logger().info('Serial bridge node started')

    def connect_serial(self):
        if not os.path.exists(self.port_name):
            self.get_logger().warn(f'Port {self.port_name} not found')
            self.ser = None
            return
        
        try:
            self.ser = serial.Serial(self.port_name, self.baud_rate, timeout=1)

            time.sleep(2)
            self.get_logger().info(
                f'Connected to ESP32 on {self.port_name} at {self.baud_rate} baud'
            )
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open {self.port_name}: {e}')
            self.ser = None

    def angles_callback(self, msg):
        # serial port must be open 
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn(
                'Serial port not open — cannot send angles.',
                throttle_duration_sec=3  # only warn once every 3 seconds
            )
            return

        if len(msg.data) != 6:
            self.get_logger().warn(
                f'Expected 6 angles, got {len(msg.data)} — skipping'
            )
            return

        # Clamp all angles to valid servo range
        angles = [max(-90.0, min(90.0, float(a))) for a in msg.data]


        # Format command string
        cmd = 'ANGLES:' + ','.join(f'{a:.2f}' for a in angles) + '\n'

        try:
            self.ser.write(cmd.encode('utf-8'))
            self.get_logger().debug(f'Sent → {cmd.strip()}')
        except serial.SerialException as e:
            self.get_logger().error(f'Write failed: {e}')
            self.ser = None   

    def read_serial(self):
        if self.ser is None or not self.ser.is_open:
            return

        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.get_logger().info(f'ESP32 → {line}')
        except serial.SerialException as e:
            self.get_logger().error(f'Read failed: {e}')
            self.ser = None


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # close serial port cleanly on exit
        if hasattr(node, 'ser') and node.ser is not None and node.ser.is_open:
            node.ser.close()
            node.get_logger().info('Serial port closed')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()