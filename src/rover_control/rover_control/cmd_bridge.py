#!/usr/bin/env python3
# ===============================
# cmd_bridge: subscribes to /cmd_vel, converts Twist -> letter,
# sends it to the ESP32 over serial.
# ===============================

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

PORT = '/dev/ttyUSB0'
BAUD = 9600

# treat values smaller than this as zero (deadzone against noise/jitter)
LINEAR_THRESHOLD = 0.05
ANGULAR_THRESHOLD = 0.05


class CmdBridge(Node):
    def __init__(self):
        super().__init__('cmd_bridge')

        # open serial to the ESP32
        self.ser = serial.Serial(PORT, BAUD, timeout=1)
        self.get_logger().info(f'Opened serial on {PORT}')

        # subscribe to /cmd_vel; cmd_callback runs on every message
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.last_cmd = None   # remember last letter sent (avoid spamming)

    def cmd_callback(self, msg):
        # -------- YOUR LOGIC GOES HERE --------
        # Decide a letter based on msg.linear.x and msg.angular.z
        # Rules to implement:
        #   angular.z clearly positive  -> 'L'
        #   angular.z clearly negative  -> 'R'
        #   linear.x clearly positive   -> 'F'
        #   linear.x clearly negative   -> 'B'
        #   otherwise                   -> 'S'
        # Use the THRESHOLD constants for "clearly".

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

        # --------------------------------------

        # only send if the command changed (don't flood the serial port)
        if cmd != self.last_cmd:
            #self.ser.write(cmd.encode())
            self.get_logger().info(f'Sent: {cmd}')
            self.last_cmd = cmd

    def destroy_node(self):
        # safety: stop the rover when the node shuts down
        self.ser.write(b'S')
        self.ser.close()
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