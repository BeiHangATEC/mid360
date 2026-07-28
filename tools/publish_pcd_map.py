#!/usr/bin/env python3
import os
import struct
import sys

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


DATATYPES = {
    ("F", 4): PointField.FLOAT32,
    ("F", 8): PointField.FLOAT64,
    ("I", 1): PointField.INT8,
    ("I", 2): PointField.INT16,
    ("I", 4): PointField.INT32,
    ("U", 1): PointField.UINT8,
    ("U", 2): PointField.UINT16,
    ("U", 4): PointField.UINT32,
}


class PcdMapPublisher(Node):
    def __init__(self, pcd_path, topic, frame_id):
        super().__init__("pcd_map_publisher")
        self.msg = self.load_pcd(pcd_path, frame_id)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(PointCloud2, topic, qos)
        self.timer = self.create_timer(1.0, self.publish_map)
        self.topic = topic
        self.get_logger().info(
            f"Loaded {self.msg.width} points from {pcd_path}; publishing {topic}"
        )

    def publish_map(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.msg)

    @staticmethod
    def load_pcd(pcd_path, frame_id):
        with open(pcd_path, "rb") as pcd_file:
            header_lines = []
            while True:
                line = pcd_file.readline()
                if not line:
                    raise RuntimeError("PCD header ended before DATA line")
                header_lines.append(line.decode("ascii").strip())
                if line.startswith(b"DATA "):
                    break
            data = pcd_file.read()

        header = {}
        for line in header_lines:
            if not line or line.startswith("#"):
                continue
            key, *values = line.split()
            header[key] = values

        if header.get("DATA", [""])[0] != "binary":
            raise RuntimeError("Only binary PCD files are supported")

        fields = header["FIELDS"]
        sizes = [int(value) for value in header["SIZE"]]
        types = header["TYPE"]
        counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
        width = int(header["WIDTH"][0])
        height = int(header.get("HEIGHT", ["1"])[0])
        point_count = int(header.get("POINTS", [str(width * height)])[0])

        ros_fields = []
        offset = 0
        for name, size, field_type, count in zip(fields, sizes, types, counts):
            datatype = DATATYPES.get((field_type, size))
            if datatype is None:
                raise RuntimeError(f"Unsupported PCD field type: {name} {field_type}{size}")
            ros_fields.append(
                PointField(name=name, offset=offset, datatype=datatype, count=count)
            )
            offset += size * count

        expected_size = point_count * offset
        if len(data) < expected_size:
            raise RuntimeError(
                f"PCD data is shorter than expected: {len(data)} < {expected_size}"
            )
        data = data[:expected_size]

        msg = PointCloud2()
        msg.header.frame_id = frame_id
        msg.header.stamp = Time()
        msg.height = height
        msg.width = width
        msg.fields = ros_fields
        msg.is_bigendian = False
        msg.point_step = offset
        msg.row_step = offset * width
        msg.data = data
        msg.is_dense = True
        return msg


def main():
    if len(sys.argv) != 4:
        print("usage: publish_pcd_map.py <map.pcd> <topic> <frame_id>", file=sys.stderr)
        return 2

    pcd_path = os.path.abspath(sys.argv[1])
    rclpy.init()
    node = PcdMapPublisher(pcd_path, sys.argv[2], sys.argv[3])
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
