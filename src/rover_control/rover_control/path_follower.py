#!/usr/bin/env python3
# path_follower: drives the rover along /planned_path via /cmd_vel.
# Turn-then-drive for skid-steer, with debug logging.
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
import math
from tf2_ros import Buffer, TransformListener, TransformException

# tuning
WAYPOINT_TOLERANCE = 0.25
GOAL_TOLERANCE = 0.20
ANGLE_TOLERANCE = 0.6       # rad (~34 deg): wider = fewer, less aggressive turns
LINEAR_SPEED = 0.3          # clear forward command (above cmd_bridge threshold)
ANGULAR_SPEED = 0.5         # clear turn command (above cmd_bridge threshold)
LOOKAHEAD = 2


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.path = []
        self.current_wp = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Path follower started.')

    def path_callback(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.current_wp = 0
        self.get_logger().info(f'New path: {len(self.path)} waypoints')

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

    def stop(self):
        self.cmd_pub.publish(Twist())

    @staticmethod
    def normalize(a):
        while a > math.pi:
            a -= 2*math.pi
        while a < -math.pi:
            a += 2*math.pi
        return a

    def control_loop(self):
        if not self.path or self.current_wp >= len(self.path):
            return
        pose = self.get_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose

        # final goal check
        gx, gy = self.path[-1]
        if math.hypot(gx - rx, gy - ry) < GOAL_TOLERANCE:
            self.stop()
            self.get_logger().info('GOAL REACHED')
            self.path = []
            return

        # advance current waypoint if reached
        while self.current_wp < len(self.path):
            wx, wy = self.path[self.current_wp]
            if math.hypot(wx - rx, wy - ry) < WAYPOINT_TOLERANCE:
                self.current_wp += 1
            else:
                break
        if self.current_wp >= len(self.path):
            self.current_wp = len(self.path) - 1

        # aim a few waypoints ahead (lookahead) for smoother tracking
        target_idx = min(self.current_wp + LOOKAHEAD, len(self.path) - 1)
        tx, ty = self.path[target_idx]

        dx, dy = tx - rx, ty - ry
        target_angle = math.atan2(dy, dx)
        angle_err = self.normalize(target_angle - ryaw)

        cmd = Twist()
        if abs(angle_err) > ANGLE_TOLERANCE:
            cmd.angular.z = ANGULAR_SPEED if angle_err > 0 else -ANGULAR_SPEED
            action = f"TURN {'L' if angle_err > 0 else 'R'}"
        else:
            cmd.linear.x = LINEAR_SPEED
            action = "FORWARD"
        self.cmd_pub.publish(cmd)

        # debug (throttled)
        self.get_logger().info(
            f'{action} | err={math.degrees(angle_err):.0f}deg wp={self.current_wp}/{len(self.path)}',
            throttle_duration_sec=0.5)

    def destroy_node(self):
        self.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
