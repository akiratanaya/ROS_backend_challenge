"""
Boustrophedon Coverage Planner Node

This node implements the area coverage system (Task 3):
1. Receives polygon areas from the user (via /coverage_area topic or service)
2. Validates areas against the map (checks reachability)
3. Generates boustrophedon (zigzag) paths for each area
4. Sends waypoints to Nav2 for execution
5. Visualizes coverage areas and planned paths in RViz
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import (
    Point, Pose, PoseStamped, PointStamped,
    Polygon, Point32,
)
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Header, ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from action_msgs.msg import GoalStatus


class CoverageArea:
    """Represents a user-defined coverage area polygon."""

    def __init__(self, area_id, polygon_points, exclude=False):
        self.area_id = area_id
        self.points = polygon_points  # list of (x, y)
        self.exclude = exclude
        self.valid = True
        self.reason = ""

    def centroid(self):
        if not self.points:
            return (0.0, 0.0)
        cx = sum(p[0] for p in self.points) / len(self.points)
        cy = sum(p[1] for p in self.points) / len(self.points)
        return (cx, cy)

    def bounding_box(self):
        xs = [p[0] for p in self.points]
        ys = [p[1] for p in self.points]
        return min(xs), min(ys), max(xs), max(ys)


class BoustrophedonPlanner(Node):
    """
    Boustrophedon coverage planner.

    Usage:
      1. Click points in RViz using "Publish Point" tool to define polygon vertices
      2. Call /finish_area service (std_msgs/String) to finalize current area
      3. Call /start_coverage service to begin coverage execution
      4. Call /clear_areas to reset all defined areas

    Topics:
      - /clicked_point (sub): Receives polygon vertices from RViz
      - /coverage_area_markers (pub): Visualization of defined areas
      - /coverage_path (pub): Planned coverage path
      - /coverage_status (pub): Status updates

    Parameters:
      - sweep_spacing: Distance between sweeplines (default: robot_width * 1.5)
      - robot_radius: Robot radius for obstacle inflation (default: 0.22)
    """

    # Color palette for areas
    AREA_COLORS = [
        (0.2, 0.6, 1.0, 0.3),   # Blue
        (0.2, 1.0, 0.4, 0.3),   # Green
        (1.0, 0.8, 0.2, 0.3),   # Yellow
        (1.0, 0.4, 0.2, 0.3),   # Orange
        (0.8, 0.2, 1.0, 0.3),   # Purple
    ]

    def __init__(self):
        super().__init__('boustrophedon_planner')

        # Parameters
        self.declare_parameter('sweep_spacing', 0.3)
        self.declare_parameter('robot_radius', 0.22)
        self.declare_parameter('map_frame', 'map')

        self.sweep_spacing = self.get_parameter('sweep_spacing').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.map_frame = self.get_parameter('map_frame').value

        # State
        self.areas = []                 # List of CoverageArea
        self.current_polygon = []       # Points being collected for current area
        self.occupancy_grid = None      # Latest map
        self.is_executing = False

        # QoS for map (transient local)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # Subscribers
        self.create_subscription(
            PointStamped, '/clicked_point', self.clicked_point_cb, 10
        )
        self.create_subscription(
            OccupancyGrid, '/map', self.map_cb, map_qos
        )

        # Publishers
        self.marker_pub = self.create_publisher(
            MarkerArray, '/coverage_area_markers', 10
        )
        self.path_pub = self.create_publisher(
            Path, '/coverage_path', 10
        )
        self.status_pub = self.create_publisher(
            String, '/coverage_status', 10
        )

        # Subscribers for commands (using String topic as simple command interface)
        self.create_subscription(
            String, '/coverage_command', self.command_cb, 10
        )

        # Nav2 action client
        self.nav_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses'
        )

        # Visualization timer
        self.create_timer(1.0, self.publish_markers)

        self.publish_status("Ready. Click points in RViz to define coverage areas.")
        self.get_logger().info("Boustrophedon Planner initialized.")
        self.get_logger().info("Commands via /coverage_command topic:")
        self.get_logger().info("  'finish_area'   - Finalize current polygon as coverage area")
        self.get_logger().info("  'exclude_area'  - Finalize current polygon as exclusion area")
        self.get_logger().info("  'start'         - Start coverage execution")
        self.get_logger().info("  'clear'         - Clear all areas")
        self.get_logger().info("  'cancel'        - Cancel current execution")

    # ─── CALLBACKS ────────────────────────────────────────────────

    def clicked_point_cb(self, msg):
        """Receive clicked points from RViz Publish Point tool."""
        x, y = msg.point.x, msg.point.y
        self.current_polygon.append((x, y))
        n = len(self.current_polygon)
        self.get_logger().info(f"Point {n} added: ({x:.2f}, {y:.2f})")
        self.publish_status(
            f"Point {n} added: ({x:.2f}, {y:.2f}). "
            f"Click more or send 'finish_area' command."
        )
        self.publish_markers()

    def map_cb(self, msg):
        """Store the latest occupancy grid map."""
        self.occupancy_grid = msg
        self.get_logger().info(
            f"Map received: {msg.info.width}x{msg.info.height}, "
            f"res={msg.info.resolution}"
        )

    def command_cb(self, msg):
        """Handle coverage commands."""
        cmd = msg.data.strip().lower()

        if cmd == 'finish_area':
            self.finish_area(exclude=False)
        elif cmd == 'exclude_area':
            self.finish_area(exclude=True)
        elif cmd == 'start':
            self.start_coverage()
        elif cmd == 'clear':
            self.clear_areas()
        elif cmd == 'cancel':
            self.cancel_coverage()
        else:
            self.get_logger().warn(f"Unknown command: {cmd}")

    # ─── AREA MANAGEMENT ─────────────────────────────────────────

    def finish_area(self, exclude=False):
        """Finalize the current polygon as a coverage or exclusion area."""
        if len(self.current_polygon) < 3:
            self.publish_status("Need at least 3 points to define an area!")
            return

        area_id = chr(ord('A') + len(self.areas))  # A, B, C, ...
        area = CoverageArea(area_id, list(self.current_polygon), exclude=exclude)

        # Validate against map
        if self.occupancy_grid is not None:
            self.validate_area(area)

        self.areas.append(area)
        self.current_polygon.clear()

        kind = "Exclusion" if exclude else "Coverage"
        status = "VALID" if area.valid else f"INVALID ({area.reason})"
        self.publish_status(f"{kind} Area {area_id} defined [{status}]")
        self.get_logger().info(f"{kind} Area {area_id}: {len(area.points)} pts, {status}")
        self.publish_markers()

    def clear_areas(self):
        """Clear all defined areas."""
        self.areas.clear()
        self.current_polygon.clear()
        self.publish_status("All areas cleared.")
        self.publish_markers()

    def validate_area(self, area):
        """Check if an area is reachable (not entirely enclosed by walls)."""
        if self.occupancy_grid is None:
            return

        grid = self.occupancy_grid
        resolution = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y
        width = grid.info.width

        # Sample points inside the polygon and check occupancy
        min_x, min_y, max_x, max_y = area.bounding_box()
        free_count = 0
        total_count = 0

        step = resolution * 2
        x = min_x
        while x <= max_x:
            y = min_y
            while y <= max_y:
                if self.point_in_polygon(x, y, area.points):
                    total_count += 1
                    gx = int((x - origin_x) / resolution)
                    gy = int((y - origin_y) / resolution)
                    if 0 <= gx < grid.info.width and 0 <= gy < grid.info.height:
                        cell = grid.data[gy * width + gx]
                        if cell == 0:  # Free space
                            free_count += 1
                y += step
            x += step

        if total_count == 0:
            area.valid = False
            area.reason = "Area too small"
        elif free_count / total_count < 0.1:
            area.valid = False
            area.reason = "Area is enclosed by walls/obstacles"
        else:
            area.valid = True

    # ─── BOUSTROPHEDON PATH GENERATION ────────────────────────────

    def generate_boustrophedon_path(self, area):
        """
        Generate a boustrophedon (zigzag/lawnmower) path for a polygon area.
        Sweeps horizontally across the polygon.
        """
        if not area.valid or area.exclude:
            return []

        min_x, min_y, max_x, max_y = area.bounding_box()
        spacing = self.sweep_spacing
        waypoints = []

        # Inflate obstacles: get occupied cells from the map
        occupied = set()
        if self.occupancy_grid is not None:
            grid = self.occupancy_grid
            res = grid.info.resolution
            ox = grid.info.origin.position.x
            oy = grid.info.origin.position.y
            w = grid.info.width

            for gy in range(grid.info.height):
                for gx in range(w):
                    cell = grid.data[gy * w + gx]
                    if cell > 50:  # Occupied
                        wx = ox + gx * res
                        wy = oy + gy * res
                        occupied.add((
                            round(wx / res) * res,
                            round(wy / res) * res
                        ))

        # Generate sweep lines (horizontal, along Y axis at fixed X intervals)
        sweep_y = min_y + spacing / 2
        direction = 1  # 1 = left-to-right, -1 = right-to-left
        row_idx = 0

        while sweep_y <= max_y - spacing / 2:
            # Find intersection of sweep line with polygon edges
            intersections = self.sweep_intersections(sweep_y, area.points)
            intersections.sort()

            # Process pairs of intersections (entry/exit)
            for i in range(0, len(intersections) - 1, 2):
                x_start = intersections[i] + self.robot_radius
                x_end = intersections[i + 1] - self.robot_radius

                if x_end <= x_start:
                    continue

                # Generate waypoints along this segment
                if direction == 1:
                    x = x_start
                    while x <= x_end:
                        if self.is_free_point(x, sweep_y, occupied, spacing / 2):
                            waypoints.append((x, sweep_y))
                        x += spacing
                    # Ensure we hit the end
                    if self.is_free_point(x_end, sweep_y, occupied, spacing / 2):
                        waypoints.append((x_end, sweep_y))
                else:
                    x = x_end
                    while x >= x_start:
                        if self.is_free_point(x, sweep_y, occupied, spacing / 2):
                            waypoints.append((x, sweep_y))
                        x -= spacing
                    if self.is_free_point(x_start, sweep_y, occupied, spacing / 2):
                        waypoints.append((x_start, sweep_y))

            direction *= -1
            sweep_y += spacing
            row_idx += 1

        return waypoints

    def sweep_intersections(self, y, polygon):
        """Find x-coordinates where a horizontal line at y intersects the polygon."""
        intersections = []
        n = len(polygon)
        for i in range(n):
            x1, y1 = polygon[i]
            x2, y2 = polygon[(i + 1) % n]

            if y1 == y2:
                continue
            if min(y1, y2) > y or max(y1, y2) < y:
                continue

            # Linear interpolation
            t = (y - y1) / (y2 - y1)
            if 0.0 <= t <= 1.0:
                x = x1 + t * (x2 - x1)
                intersections.append(x)

        return intersections

    def is_free_point(self, x, y, occupied_set, radius):
        """Check if a point is free from obstacles (simple grid check)."""
        if self.occupancy_grid is None:
            return True

        res = self.occupancy_grid.info.resolution
        ox = self.occupancy_grid.info.origin.position.x
        oy = self.occupancy_grid.info.origin.position.y
        w = self.occupancy_grid.info.width
        h = self.occupancy_grid.info.height

        # Check cells within robot radius
        cells_to_check = int(math.ceil(radius / res))
        gx_c = int((x - ox) / res)
        gy_c = int((y - oy) / res)

        for dy in range(-cells_to_check, cells_to_check + 1):
            for dx in range(-cells_to_check, cells_to_check + 1):
                gx = gx_c + dx
                gy = gy_c + dy
                if 0 <= gx < w and 0 <= gy < h:
                    cell = self.occupancy_grid.data[gy * w + gx]
                    if cell > 50:  # Occupied or unknown
                        return False
        return True

    @staticmethod
    def point_in_polygon(x, y, polygon):
        """Ray casting algorithm for point-in-polygon test."""
        n = len(polygon)
        inside = False
        j = n - 1
        for i in range(n):
            xi, yi = polygon[i]
            xj, yj = polygon[j]
            if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
                inside = not inside
            j = i
        return inside

    # ─── COVERAGE EXECUTION ───────────────────────────────────────

    def start_coverage(self):
        """Plan and execute coverage for all defined areas."""
        if self.is_executing:
            self.publish_status("Coverage already in progress!")
            return

        valid_areas = [a for a in self.areas if a.valid and not a.exclude]
        if not valid_areas:
            self.publish_status("No valid coverage areas defined!")
            return

        self.publish_status(f"Planning coverage for {len(valid_areas)} area(s)...")

        # Optional: sort areas by distance to robot (nearest first)
        # For now, process in definition order
        all_waypoints = []
        for area in valid_areas:
            wps = self.generate_boustrophedon_path(area)
            self.get_logger().info(
                f"Area {area.area_id}: {len(wps)} waypoints generated"
            )
            all_waypoints.extend(wps)

        if not all_waypoints:
            self.publish_status("No waypoints generated! Check area validity.")
            return

        # Publish path for visualization
        self.publish_coverage_path(all_waypoints)
        self.publish_status(
            f"Coverage path: {len(all_waypoints)} waypoints. Sending to Nav2..."
        )

        # Send to Nav2
        self.execute_coverage(all_waypoints)

    def execute_coverage(self, waypoints):
        """Send waypoints to Nav2 NavigateThroughPoses."""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.publish_status("Nav2 action server not available!")
            self.get_logger().error("NavigateThroughPoses action server not available")
            return

        goal = NavigateThroughPoses.Goal()
        for (x, y) in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            # Orientation: face direction of travel
            pose.pose.orientation.w = 1.0
            goal.poses.append(pose)

        self.is_executing = True
        future = self.nav_client.send_goal_async(
            goal, feedback_callback=self.nav_feedback_cb
        )
        future.add_done_callback(self.nav_goal_response_cb)

    def nav_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.publish_status("Coverage goal rejected by Nav2!")
            self.is_executing = False
            return

        self.publish_status("Coverage execution started...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_cb)

    def nav_feedback_cb(self, feedback_msg):
        remaining = feedback_msg.feedback.number_of_poses_remaining
        self.publish_status(f"Covering... {remaining} waypoints remaining")

    def nav_result_cb(self, future):
        result = future.result()
        self.is_executing = False
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.publish_status("Coverage complete!")
        else:
            self.publish_status(f"Coverage ended with status: {result.status}")

    def cancel_coverage(self):
        """Cancel current coverage execution."""
        if not self.is_executing:
            self.publish_status("No coverage in progress.")
            return
        self.publish_status("Cancelling coverage...")
        self.is_executing = False

    # ─── VISUALIZATION ────────────────────────────────────────────

    def publish_markers(self):
        """Publish area polygon markers and current points for RViz."""
        marker_array = MarkerArray()
        marker_id = 0

        # Delete all previous markers
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.header.frame_id = self.map_frame
        delete_marker.id = marker_id
        marker_array.markers.append(delete_marker)
        marker_id += 1

        # Draw finalized areas
        for i, area in enumerate(self.areas):
            color_idx = i % len(self.AREA_COLORS)
            r, g, b, a = self.AREA_COLORS[color_idx]

            if area.exclude:
                r, g, b, a = 1.0, 0.0, 0.0, 0.2  # Red for exclusion

            if not area.valid:
                a = 0.1  # Dim invalid areas

            # Filled polygon (triangle list approximation)
            poly_marker = Marker()
            poly_marker.header.frame_id = self.map_frame
            poly_marker.header.stamp = self.get_clock().now().to_msg()
            poly_marker.ns = "coverage_areas"
            poly_marker.id = marker_id
            poly_marker.type = Marker.LINE_STRIP
            poly_marker.action = Marker.ADD
            poly_marker.scale.x = 0.05
            poly_marker.color = ColorRGBA(r=r, g=g, b=b, a=min(a + 0.5, 1.0))
            poly_marker.pose.orientation.w = 1.0

            for pt in area.points:
                p = Point(x=pt[0], y=pt[1], z=0.05)
                poly_marker.points.append(p)
            # Close the polygon
            if area.points:
                p = Point(x=area.points[0][0], y=area.points[0][1], z=0.05)
                poly_marker.points.append(p)

            marker_array.markers.append(poly_marker)
            marker_id += 1

            # Area label
            label = Marker()
            label.header.frame_id = self.map_frame
            label.header.stamp = self.get_clock().now().to_msg()
            label.ns = "area_labels"
            label.id = marker_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            cx, cy = area.centroid()
            label.pose.position = Point(x=cx, y=cy, z=0.3)
            label.pose.orientation.w = 1.0
            label.scale.z = 0.4
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            kind = "EXCL" if area.exclude else ""
            valid_str = "" if area.valid else " [INVALID]"
            label.text = f"Area {area.area_id} {kind}{valid_str}"
            marker_array.markers.append(label)
            marker_id += 1

        # Draw current polygon being defined
        if self.current_polygon:
            curr_marker = Marker()
            curr_marker.header.frame_id = self.map_frame
            curr_marker.header.stamp = self.get_clock().now().to_msg()
            curr_marker.ns = "current_polygon"
            curr_marker.id = marker_id
            curr_marker.type = Marker.LINE_STRIP
            curr_marker.action = Marker.ADD
            curr_marker.scale.x = 0.03
            curr_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)
            curr_marker.pose.orientation.w = 1.0

            for pt in self.current_polygon:
                p = Point(x=pt[0], y=pt[1], z=0.05)
                curr_marker.points.append(p)

            marker_array.markers.append(curr_marker)
            marker_id += 1

            # Vertex spheres
            for j, pt in enumerate(self.current_polygon):
                sphere = Marker()
                sphere.header.frame_id = self.map_frame
                sphere.header.stamp = self.get_clock().now().to_msg()
                sphere.ns = "current_vertices"
                sphere.id = marker_id
                sphere.type = Marker.SPHERE
                sphere.action = Marker.ADD
                sphere.pose.position = Point(x=pt[0], y=pt[1], z=0.05)
                sphere.pose.orientation.w = 1.0
                sphere.scale.x = 0.1
                sphere.scale.y = 0.1
                sphere.scale.z = 0.1
                sphere.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
                marker_array.markers.append(sphere)
                marker_id += 1

        self.marker_pub.publish(marker_array)

    def publish_coverage_path(self, waypoints):
        """Publish the planned coverage path for RViz visualization."""
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()

        for (x, y) in waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.path_pub.publish(path)

    def publish_status(self, msg_str):
        """Publish status string."""
        msg = String()
        msg.data = msg_str
        self.status_pub.publish(msg)
        self.get_logger().info(f"[STATUS] {msg_str}")


def main(args=None):
    rclpy.init(args=args)
    node = BoustrophedonPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
