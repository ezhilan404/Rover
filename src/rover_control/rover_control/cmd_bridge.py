#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

PORT = '/dev/rover_esp32'
BAUD = 9600
LINEAR_THRESHOLD = 0.05
ANGULAR_THRESHOLD = 0.05


class CmdBridge(Node):
    def __init__(self):
        super().__init__('cmd_bridge')
        self.ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(2)
        self.get_logger().info(f'Opened serial on {PORT}')
        self.current_cmd = 'S'
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_timer(0.2, self.send_current)

    def cmd_callback(self, msg):
        if msg.angular.z > ANGULAR_THRESHOLD:
            cmd = 'L'
        elif msg.angular.z < -ANGULAR_THRESHOLD:
            cmd = 'R'
        elif msg.linear.x > LINEAR_THRESHOLD:
            cmd = 'F'
        elif msg.linear.x < -LINEAR_THRESHOLD:
            cmd = 'B'
        else:
            cmd = 'S'
        self.current_cmd = cmd

    def send_current(self):
        self.ser.write(self.current_cmd.encode())

    def destroy_node(self):
        try:
            self.ser.write(b'S')
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
