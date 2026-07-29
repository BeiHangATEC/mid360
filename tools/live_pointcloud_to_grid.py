#!/usr/bin/env python3
import math
import sys

import rclpy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class LivePointCloudToGrid(Node):
    def __init__(self):
        super().__init__("live_pointcloud_to_grid")
        self.declare_parameter("input_topic", "/cloud_registered")
        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("frame_id", "camera_init")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("min_z", -0.8)
        self.declare_parameter("max_z", 0.8)
        self.declare_parameter("publish_rate", 2.0)
        self.declare_parameter("margin_cells", 20)
        self.declare_parameter("raytrace", True)
        self.declare_parameter("max_range", 30.0)
        self.declare_parameter("process_every_n", 1)
        self.declare_parameter("point_stride", 1)

        self.input_topic = self.get_parameter("input_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.map_topic = self.get_parameter("map_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.resolution = float(self.get_parameter("resolution").value)
        self.min_z = float(self.get_parameter("min_z").value)
        self.max_z = float(self.get_parameter("max_z").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.margin_cells = int(self.get_parameter("margin_cells").value)
        self.raytrace = bool(self.get_parameter("raytrace").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.process_every_n = max(1, int(self.get_parameter("process_every_n").value))
        self.point_stride = max(1, int(self.get_parameter("point_stride").value))

        self.free_cells = set()
        self.occupied_cells = set()
        self.min_ix = None
        self.max_ix = None
        self.min_iy = None
        self.max_iy = None
        self.last_added = 0
        self.sensor_xy = None
        self.warned_no_odom = False
        self.cloud_count = 0

        sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.odom_subscription = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, sub_qos
        )
        self.subscription = self.create_subscription(
            PointCloud2, self.input_topic, self.cloud_callback, sub_qos
        )
        self.publisher = self.create_publisher(OccupancyGrid, self.map_topic, 1)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_grid)

        self.get_logger().info(
            "Projecting %s to %s at %.3fm resolution, z=[%.2f, %.2f], raytrace=%s, process_every_n=%d, point_stride=%d"
            % (
                self.input_topic,
                self.map_topic,
                self.resolution,
                self.min_z,
                self.max_z,
                self.raytrace,
                self.process_every_n,
                self.point_stride,
            )
        )

    def odom_callback(self, msg):
        self.sensor_xy = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )

    @staticmethod
    def bresenham(start_ix, start_iy, end_ix, end_iy):
        x0, y0 = start_ix, start_iy
        x1, y1 = end_ix, end_iy
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy

        while True:
            yield x0, y0
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy

    def include_cell_in_bounds(self, ix, iy):
        if self.min_ix is None:
            self.min_ix = self.max_ix = ix
            self.min_iy = self.max_iy = iy
            return
        self.min_ix = min(self.min_ix, ix)
        self.max_ix = max(self.max_ix, ix)
        self.min_iy = min(self.min_iy, iy)
        self.max_iy = max(self.max_iy, iy)

    def mark_free(self, ix, iy):
        cell = (ix, iy)
        if cell not in self.occupied_cells:
            self.free_cells.add(cell)
        self.include_cell_in_bounds(ix, iy)

    def mark_occupied(self, ix, iy):
        cell = (ix, iy)
        self.occupied_cells.add(cell)
        self.free_cells.discard(cell)
        self.include_cell_in_bounds(ix, iy)

    def cloud_callback(self, msg):
        self.cloud_count += 1
        if self.cloud_count % self.process_every_n != 0:
            return

        if self.raytrace and self.sensor_xy is None:
            if not self.warned_no_odom:
                self.get_logger().warn("Waiting for odometry before raytracing /map")
                self.warned_no_odom = True
            return

        added = 0
        if self.sensor_xy is None:
            sensor_ix = sensor_iy = None
        else:
            sensor_ix = math.floor(self.sensor_xy[0] / self.resolution)
            sensor_iy = math.floor(self.sensor_xy[1] / self.resolution)

        for row_index, row in enumerate(point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )):
            if row_index % self.point_stride != 0:
                continue

            if hasattr(row, "dtype") and row.dtype.names:
                x = float(row["x"])
                y = float(row["y"])
                z = float(row["z"])
            else:
                x, y, z = float(row[0]), float(row[1]), float(row[2])

            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            if z < self.min_z or z > self.max_z:
                continue
            if self.sensor_xy is not None:
                distance = math.hypot(x - self.sensor_xy[0], y - self.sensor_xy[1])
                if distance > self.max_range:
                    continue

            ix = math.floor(x / self.resolution)
            iy = math.floor(y / self.resolution)
            cell = (ix, iy)
            if self.raytrace and sensor_ix is not None and sensor_iy is not None:
                prev_cell = None
                for ray_ix, ray_iy in self.bresenham(sensor_ix, sensor_iy, ix, iy):
                    if prev_cell is not None:
                        self.mark_free(prev_cell[0], prev_cell[1])
                    prev_cell = (ray_ix, ray_iy)
                if cell not in self.occupied_cells:
                    added += 1
                self.mark_occupied(ix, iy)
            elif cell not in self.occupied_cells:
                self.mark_occupied(ix, iy)
                added += 1
        self.last_added = added

    def publish_grid(self):
        if self.min_ix is None:
            return

        min_ix = self.min_ix - self.margin_cells
        max_ix = self.max_ix + self.margin_cells
        min_iy = self.min_iy - self.margin_cells
        max_iy = self.max_iy + self.margin_cells
        width = max_ix - min_ix + 1
        height = max_iy - min_iy + 1

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.frame_id
        grid.info.resolution = self.resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = min_ix * self.resolution
        grid.info.origin.position.y = min_iy * self.resolution
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0

        data = [-1] * (width * height)
        for ix, iy in self.free_cells:
            if min_ix <= ix <= max_ix and min_iy <= iy <= max_iy:
                data[(iy - min_iy) * width + (ix - min_ix)] = 0
        for ix, iy in self.occupied_cells:
            if min_ix <= ix <= max_ix and min_iy <= iy <= max_iy:
                data[(iy - min_iy) * width + (ix - min_ix)] = 100
        grid.data = data
        try:
            self.publisher.publish(grid)
        except Exception:
            if rclpy.ok():
                raise


def main():
    rclpy.init(args=sys.argv)
    node = LivePointCloudToGrid()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
