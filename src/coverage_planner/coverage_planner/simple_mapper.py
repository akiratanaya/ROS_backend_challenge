#!/usr/bin/env python3
"""
Simple 2D Occupancy Grid Mapper.
Subscribes to /scan and uses TF (odom->base_scan) to build a map.
Publishes to /map as nav_msgs/OccupancyGrid.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time
import tf2_ros
from tf2_ros import TransformException


class SimpleMapper(Node):
    def __init__(self):
        super().__init__('simple_mapper')

        # Declare parameters
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('map_width', 300)   # cells (300 * 0.05 = 15m)
        self.declare_parameter('map_height', 300)
        self.declare_parameter('map_origin_x', -7.5)
        self.declare_parameter('map_origin_y', -7.5)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_update_interval', 1.0)
        self.declare_parameter('max_laser_range', 12.0)

        # Get parameters
        self.resolution = self.get_parameter('resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.origin_x = self.get_parameter('map_origin_x').value
        self.origin_y = self.get_parameter('map_origin_y').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        scan_topic = self.get_parameter('scan_topic').value
        map_interval = self.get_parameter('map_update_interval').value
        self.max_range = self.get_parameter('max_laser_range').value

        # Log-odds map (0 = unknown, positive = occupied, negative = free)
        self.log_odds = np.zeros((self.map_height, self.map_width), dtype=np.float32)
        self.l_occ = 0.85    # log-odds increment for occupied
        self.l_free = -0.40  # log-odds increment for free
        self.l_max = 5.0
        self.l_min = -5.0

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Also broadcast map->odom (identity, since we treat odom as map frame)
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self._publish_map_to_odom_tf()

        # Subscriber - use best effort QoS to match the bridge
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, scan_qos
        )

        # Publisher
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)

        # Timer for publishing map
        self.map_timer = self.create_timer(map_interval, self.publish_map)

        self.scan_count = 0
        self.get_logger().info(
            f'SimpleMapper started: {self.map_width}x{self.map_height} @ {self.resolution}m/cell, '
            f'listening on {scan_topic}'
        )

    def _publish_map_to_odom_tf(self):
        """Publish static identity transform from map -> odom."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f'Published static TF: {self.map_frame} -> {self.odom_frame}')

    def world_to_map(self, wx, wy):
        """Convert world coordinates to map cell indices."""
        mx = int((wx - self.origin_x) / self.resolution)
        my = int((wy - self.origin_y) / self.resolution)
        return mx, my

    def in_bounds(self, mx, my):
        return 0 <= mx < self.map_width and 0 <= my < self.map_height

    def bresenham(self, x0, y0, x1, y1):
        """Bresenham's line algorithm to get cells along a ray."""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return cells

    def scan_callback(self, msg: LaserScan):
        """Process incoming laser scan and update the occupancy grid."""
        # Look up the transform from odom to the scan frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                msg.header.frame_id,
                rclpy.time.Time(),  # Use latest available
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except TransformException as e:
            if self.scan_count == 0:
                self.get_logger().warn(f'TF lookup failed: {e}')
            return

        # Extract robot position and yaw from transform
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        q = transform.transform.rotation
        # Compute yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Get sensor position in map cells
        sx, sy = self.world_to_map(tx, ty)

        self.scan_count += 1
        if self.scan_count % 5 == 1:
            self.get_logger().info(
                f'Scan #{self.scan_count}: robot at ({tx:.2f}, {ty:.2f}), '
                f'yaw={math.degrees(yaw):.1f}°, {len(msg.ranges)} rays'
            )

        # Process each ray
        angle = msg.angle_min
        for i, r in enumerate(msg.ranges):
            angle_i = angle + i * msg.angle_increment

            # Skip invalid ranges
            if r < msg.range_min or r > msg.range_max or math.isnan(r) or math.isinf(r):
                continue

            # Clamp to max usable range
            effective_range = min(r, self.max_range)

            # Endpoint of the ray in world coordinates
            hit_x = tx + effective_range * math.cos(yaw + angle_i)
            hit_y = ty + effective_range * math.sin(yaw + angle_i)

            # Convert to map cells
            ex, ey = self.world_to_map(hit_x, hit_y)

            # Trace free cells along the ray (Bresenham)
            ray_cells = self.bresenham(sx, sy, ex, ey)

            # Mark all cells except the last as free
            for cx, cy in ray_cells[:-1]:
                if self.in_bounds(cx, cy):
                    self.log_odds[cy, cx] = max(
                        self.log_odds[cy, cx] + self.l_free, self.l_min
                    )

            # Mark the endpoint as occupied (only if within max range)
            if r < self.max_range and self.in_bounds(ex, ey):
                self.log_odds[ey, ex] = min(
                    self.log_odds[ey, ex] + self.l_occ, self.l_max
                )

    def publish_map(self):
        """Convert log-odds to OccupancyGrid and publish."""
        if self.scan_count == 0:
            return  # Don't publish empty map

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.map_frame

        grid.info = MapMetaData()
        grid.info.resolution = self.resolution
        grid.info.width = self.map_width
        grid.info.height = self.map_height
        grid.info.origin.position.x = self.origin_x
        grid.info.origin.position.y = self.origin_y
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0

        # Convert log-odds to probability [0, 100] or -1 (unknown)
        data = np.full(self.map_width * self.map_height, -1, dtype=np.int8)
        flat = self.log_odds.flatten()

        # Occupied: log_odds > 0.5
        occupied_mask = flat > 0.5
        data[occupied_mask] = 100

        # Free: log_odds < -0.5
        free_mask = flat < -0.5
        data[free_mask] = 0

        grid.data = data.tolist()
        self.map_pub.publish(grid)

    def destroy_node(self):
        self.get_logger().info(f'Mapper shutting down. Processed {self.scan_count} scans.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
