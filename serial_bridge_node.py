import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
import serial
import json

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        # فتح الـ serial port
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
            self.get_logger().info('Serial port opened')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        self.latest_direction = 'SEARCH'  # آخر direction وصل

        # Subscribe لـ position
        self.sub_pos = self.create_subscription(
            Point, '/ball_position', self.position_callback, 10)

        # Subscribe لـ direction
        self.sub_dir = self.create_subscription(
            String, '/ball_direction', self.direction_callback, 10)

    def direction_callback(self, msg):
        self.latest_direction = msg.data

    def position_callback(self, msg):
        data = {
            'x': round(msg.x, 3),
            'y': round(msg.y, 3),
            'w': round(msg.z, 1),
            'd': self.latest_direction      # ← اتضاف
        }
        json_str = json.dumps(data) + '\n'

        try:
            self.ser.write(json_str.encode('utf-8'))
            self.get_logger().info(f'Sent: {json_str.strip()}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write failed: {e}')

    def destroy_node(self):
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()