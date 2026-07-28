#!/usr/bin/env python3
import math
import os
import struct
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class PointCloudPcdSaver(Node):
    def __init__(self, topic, output_path):
        super().__init__("pointcloud2_pcd_saver")
        self.topic = topic
        self.output_path = output_path
        self.done = False
        self.subscription = self.create_subscription(
            PointCloud2, topic, self.save_message, 10
        )

    def save_message(self, msg):
        field_names = [field.name for field in msg.fields]
        if "x" not in field_names or "y" not in field_names or "z" not in field_names:
            raise RuntimeError(f"{self.topic} does not contain x/y/z fields")

        selected_fields = ["x", "y", "z"]
        has_intensity = "intensity" in field_names
        if has_intensity:
            selected_fields.append("intensity")

        points = []
        for row in point_cloud2.read_points(
            msg, field_names=selected_fields, skip_nans=True
        ):
            if hasattr(row, "dtype") and row.dtype.names:
                values = [row[name].item() for name in selected_fields]
            else:
                values = list(row)

            x, y, z = (float(values[0]), float(values[1]), float(values[2]))
            intensity = float(values[3]) if has_intensity else 0.0
            if all(math.isfinite(value) for value in (x, y, z, intensity)):
                points.append((x, y, z, intensity))

        os.makedirs(os.path.dirname(os.path.abspath(self.output_path)), exist_ok=True)
        header = (
            "# .PCD v0.7 - Point Cloud Data file format\n"
            "VERSION 0.7\n"
            "FIELDS x y z intensity\n"
            "SIZE 4 4 4 4\n"
            "TYPE F F F F\n"
            "COUNT 1 1 1 1\n"
            f"WIDTH {len(points)}\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {len(points)}\n"
            "DATA binary\n"
        )

        with open(self.output_path, "wb") as pcd_file:
            pcd_file.write(header.encode("ascii"))
            for point in points:
                pcd_file.write(struct.pack("<ffff", *point))

        print(f"saved {len(points)} points to {self.output_path}", flush=True)
        self.done = True


def main():
    if len(sys.argv) != 3:
        print(
            "usage: save_pointcloud2_pcd.py <pointcloud2_topic> <output.pcd>",
            file=sys.stderr,
        )
        return 2

    rclpy.init()
    node = PointCloudPcdSaver(sys.argv[1], sys.argv[2])
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
