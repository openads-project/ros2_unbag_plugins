# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

from pathlib import Path

from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2

from ros2_unbag.core.routines.base import ExportMetadata, ExportMode, ExportRoutine
from ros2_unbag.core.routines.pointcloud import (
    export_pointcloud_pcd as _export_pointcloud_pcd,
    export_pointcloud_pkl as _export_pointcloud_pkl,
    export_pointcloud_xyz as _export_pointcloud_xyz,
)


@ExportRoutine(
    "point_cloud_interfaces/msg/CompressedPointCloud2",
    ["pointcloud/pkl"],
    mode=ExportMode.MULTI_FILE,
)
def export_cloudini_pointcloud_pkl(msg, path: Path, fmt: str, metadata: ExportMetadata):
    decompressed = decode_cloudini_compressed_pointcloud(msg)
    _export_pointcloud_pkl(decompressed, path, fmt, metadata)


@ExportRoutine(
    "point_cloud_interfaces/msg/CompressedPointCloud2",
    ["pointcloud/xyz"],
    mode=ExportMode.MULTI_FILE,
)
def export_cloudini_pointcloud_xyz(msg, path: Path, fmt: str, metadata: ExportMetadata):
    decompressed = decode_cloudini_compressed_pointcloud(msg)
    _export_pointcloud_xyz(decompressed, path, fmt, metadata)


@ExportRoutine(
    "point_cloud_interfaces/msg/CompressedPointCloud2",
    ["pointcloud/pcd", "pointcloud/pcd_compressed", "pointcloud/pcd_ascii"],
    mode=ExportMode.MULTI_FILE,
)
def export_cloudini_pointcloud_pcd(msg, path: Path, fmt: str, metadata: ExportMetadata):
    decompressed = decode_cloudini_compressed_pointcloud(msg)
    _export_pointcloud_pcd(decompressed, path, fmt, metadata)


__all__ = ["decode_cloudini_compressed_pointcloud"]


def decode_cloudini_compressed_pointcloud(msg) -> PointCloud2:
    """Decode with the installed upstream Cloudini library (including its version checks)."""
    if isinstance(msg, PointCloud2):
        return msg
    if getattr(msg, "format", "cloudini") not in ("cloudini", ""):
        raise ValueError(f"Unsupported compressed point cloud format: {msg.format}")
    if not hasattr(msg, "compressed_data"):
        raise AttributeError("Message does not contain 'compressed_data'; expected Cloudini-compressed input")

    # Routine installation happens before colcon builds the native module. Import
    # lazily so registration and unrelated export routines still work at that stage.
    try:
        from _ros2_unbag_cloudini import decode
    except ImportError as exc:
        raise RuntimeError(
            "Cloudini native decoder is unavailable. Run python3 -m pip install . "
            "from the ros2_unbag_plugins checkout, or build the ros2_unbag_cloudini "
            "colcon package and source the workspace's install/setup.bash."
        ) from exc
    return deserialize_message(decode(serialize_message(msg)), PointCloud2)
