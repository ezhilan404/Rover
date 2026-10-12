#!/usr/bin/env python3
# a_star_planner: risk-weighted A* over the live /risk_heatmap.
# Start = rover's current pose (TF). Goal = /goal_pose (RViz "2D Goal Pose").
# Publishes /planned_path (nav_msgs/Path).
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import heapq
import math

from tf2_ros import Buffer, TransformListener, TransformException

RISK_WEIGHT = 2.0

# --- FIX #3 -----------------------------------------------------------
# Your fusion scoring (obstacle +2, fire +10, gas +8, instability +7,
# human +3) tops out around 30 for a single cell, so the old
# OBSTACLE_THRESHOLD = 50 could basically never fire -- real rubble/
# walls were never hard-blocked, just soft-penalized like any other
# risk. We now separate two concepts:
#
#   HARD_BLOCK_VALUE   -> cells the fusion node marks as a literal,
#                         physically-impassable obstacle. If your
#                         /risk_heatmap publishes the standard
#                         OccupancyGrid convention (100 = occupied) for
#                         those cells *in addition to* the additive
#                         risk score, keep this at 90. If instead the
#                         "+2 obstacle" contribution IS how obstacles
#                         are marked and there's no separate hard value,
#                         set HARD_BLOCK_VALUE = None and the planner
#                         will fall back to only ever soft-penalizing
#                         obstacles -- confirm this against your
#                         risk_heatmap fusion node before flying it.
#   DANGER_BLOCK_VALUE -> soft risk score high enough that we treat the
#                         cell as effectively impassable (matches your
#                         own "danger" threshold, risk >= 7, but we set
#                         it a bit above that so genuinely narrow
#                         mandatory-danger passages are still plannable
#                         in an emergency rather than silently failing).
# ------------------------------------------------------------------
HARD_BLOCK_VALUE = 90
DANGER_BLOCK_VALUE = 25


class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')

        self.risk_grid = None
        self.map_info = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(OccupancyGrid, '/risk_heatmap', self.grid_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        self.get_logger().info('A* planner started. Set a goal with "2D Goal Pose" in RViz.')

    def grid_callback(self, msg):
        self.map_info = msg.info
        h, w = msg.info.height, msg.info.width
        self.risk_grid = np.array(msg.data, dtype=np.int16).reshape(h, w)

    # --- FIX #1 ---------------------------------------------------
    # int() truncates toward zero, so a point 0.3m *behind* the map
    # origin (very common -- the origin is rarely exactly at the
    # rover's start) rounded to cell 0 instead of cell -1, silently
    # misplacing the start/goal by up to one cell. math.floor() is the
    # correct conversion for a row/col grid.
    # ---------------------------------------------------------------
    def world_to_cell(self, x, y):
        col = math.floor((x - self.map_info.origin.position.x) / self.map_info.resolution)
        row = math.floor((y - self.map_info.origin.position.y) / self.map_info.resolution)
        return (row, col)

    def cell_to_world(self, row, col):
        # +0.5 cell so the published pose sits at the cell CENTER, not
        # its corner -- keeps the follower from clipping cell edges.
        x = (col + 0.5) * self.map_info.resolution + self.map_info.origin.position.x
        y = (row + 0.5) * self.map_info.resolution + self.map_info.origin.position.y
        return (x, y)

    def get_start_cell(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warning(f'No map->base_link TF yet: {ex}')
            return None
        return self.world_to_cell(t.transform.translation.x, t.transform.translation.y)

    def in_bounds(self, cell):
        h, w = self.risk_grid.shape
        return 0 <= cell[0] < h and 0 <= cell[1] < w

    def goal_callback(self, msg):
        if self.risk_grid is None or self.map_info is None:
            self.get_logger().warning('No heatmap yet — cannot plan.')
            return
        start = self.get_start_cell()
        if start is None:
            self.get_logger().warning('No rover pose (map->base_link) — cannot plan.')
            return
        goal = self.world_to_cell(msg.pose.position.x, msg.pose.position.y)

        h, w = self.risk_grid.shape
        self.get_logger().info(f'grid={w}x{h}, start={start}, goal={goal}')

        # --- FIX #2 -----------------------------------------------
        # The old code logged "OUT OF BOUNDS" but still called
        # self.astar(start, goal) with the bad cell. NumPy silently
        # wraps negative indices instead of raising, so a drifted TF
        # or a goal clicked outside the map produced a plan from a
        # wrapped-around, nonsense start cell instead of failing
        # loudly. Now we actually abort.
        # ------------------------------------------------------------
        if not self.in_bounds(start):
            self.get_logger().error(f'START {start} is out of bounds for a {w}x{h} grid — aborting plan.')
            return
        if not self.in_bounds(goal):
            self.get_logger().error(f'GOAL {goal} is out of bounds for a {w}x{h} grid — aborting plan.')
            return

        self.get_logger().info(f'start risk={self.risk_grid[start[0], start[1]]}, goal risk={self.risk_grid[goal[0], goal[1]]}')

        path_cells = self.astar(start, goal)
        if path_cells is None:
            self.get_logger().warning('No path found.')
            return

        self.publish_path(path_cells)
        self.get_logger().info(f'Path planned: {len(path_cells)} cells.')

    def astar(self, start, goal):
        grid = self.risk_grid
        rows, cols = grid.shape

        def h(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        closed = set()  # FIX #4: avoid re-expanding already-finalized nodes

        while open_set:
            _, current = heapq.heappop(open_set)
            if current in closed:
                continue
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[::-1]
            closed.add(current)

            r, c = current
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < rows and 0 <= nc < cols):
                    continue
                neighbor = (nr, nc)
                if neighbor in closed:
                    continue

                cell_value = int(grid[nr, nc])

                if cell_value < 0:  # -1 = unknown -> treat as free
                    cell_risk = 0
                elif HARD_BLOCK_VALUE is not None and cell_value >= HARD_BLOCK_VALUE:
                    continue  # physically impassable, never enter
                elif cell_value >= DANGER_BLOCK_VALUE:
                    continue  # effectively impassable risk (fire/gas/etc.)
                else:
                    cell_risk = cell_value

                tentative_g = g_score[current] + 1 + cell_risk * RISK_WEIGHT
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + h(neighbor, goal)
                    heapq.heappush(open_set, (f, neighbor))
        return None

    def publish_path(self, cells):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for (row, col) in cells:
            x, y = self.cell_to_world(row, col)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
