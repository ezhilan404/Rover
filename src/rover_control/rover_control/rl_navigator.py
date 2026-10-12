#!/usr/bin/env python3
# rl_navigator: navigates using a trained Q-table (learned policy).
# Reads /risk_heatmap + rover pose + goal, computes the same state as
# training, looks up the best action, publishes /cmd_vel.
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
import math
import os

from tf2_ros import Buffer, TransformListener, TransformException

# --- must match training constants ---
RISK_MED = 3
RISK_DANGER = 7
BLOCKED = 100
# actions order (must match training): up, down, left, right in MAP frame
ACTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]

GOAL_TOLERANCE = 0.25       # m
LINEAR_SPEED = 0.2
ANGULAR_SPEED = 0.5

QTABLE_PATH = os.path.expanduser('~/Rover/rover_qtable.npy')


class RLNavigator(Node):
    def __init__(self):
        super().__init__('rl_navigator')

        self.Q = np.load(QTABLE_PATH)
        self.get_logger().info(f'Loaded Q-table {self.Q.shape} from {QTABLE_PATH}')

        self.risk_grid = None
        self.map_info = None
        self.goal = None       # (x, y) in map

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(OccupancyGrid, '/risk_heatmap', self.grid_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(0.3, self.control_loop)
        self.get_logger().info('RL navigator started. Set a goal in RViz.')

    def grid_cb(self, msg):
        self.map_info = msg.info
        h, w = msg.info.height, msg.info.width
        self.risk_grid = np.array(msg.data, dtype=np.int16).reshape(h, w)

    def goal_cb(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f'Goal set: {self.goal}')

    def world_to_cell(self, x, y):
        col = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        row = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        return (row, col)

    def get_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException:
            return None
        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        return (x, y, yaw)

    def risk_level(self, v):
        if v < 0:      # unknown
            return 0
        if v >= RISK_DANGER:
            return 2
        if v >= RISK_MED:
            return 1
        return 0

    def goal_sector(self, pos_cell, goal_cell):
        dr = goal_cell[0] - pos_cell[0]
        dc = goal_cell[1] - pos_cell[1]
        angle = np.arctan2(dc, -dr)
        return int(round(angle / (np.pi/4))) % 8

    def get_state(self, pos_cell, goal_cell):
        r, c = pos_cell
        h, w = self.risk_grid.shape
        levels = []
        for dr, dc in ACTIONS:
            nr, nc = r+dr, c+dc
            if not (0 <= nr < h and 0 <= nc < w):
                levels.append(2)
            else:
                levels.append(self.risk_level(self.risk_grid[nr, nc]))
        idx = 0
        for lv in levels:
            idx = idx*3 + lv
        idx = idx*8 + self.goal_sector(pos_cell, goal_cell)
        return idx

    def stop(self):
        self.cmd_pub.publish(Twist())

    def control_loop(self):
        if self.goal is None or self.risk_grid is None or self.map_info is None:
            return
        pose = self.get_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose

        # reached goal?
        if math.hypot(self.goal[0]-rx, self.goal[1]-ry) < GOAL_TOLERANCE:
            self.stop()
            self.get_logger().info('Goal reached (RL).')
            self.goal = None
            return

        pos_cell = self.world_to_cell(rx, ry)
        goal_cell = self.world_to_cell(self.goal[0], self.goal[1])

        # look up best action from the learned policy
        state = self.get_state(pos_cell, goal_cell)
        action = int(np.argmax(self.Q[state]))

        # action is a MAP-frame direction; convert to a heading, then drive
        dr, dc = ACTIONS[action]
        # desired heading in map frame (row up = -x world-ish; match training convention)
        target_angle = math.atan2(dc, -dr)
        angle_err = self.normalize(target_angle - ryaw)

        cmd = Twist()
        if abs(angle_err) > 0.35:
            cmd.angular.z = ANGULAR_SPEED if angle_err > 0 else -ANGULAR_SPEED
        else:
            cmd.linear.x = LINEAR_SPEED
        self.cmd_pub.publish(cmd)

    @staticmethod
    def normalize(a):
        while a > math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

    def destroy_node(self):
        self.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RLNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
