# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

import numpy as np

import matplotlib.colors as mcolors
import matplotlib.pyplot as plt

from ros2_unbag.core.processors.base import Processor
from sensor_msgs.msg import PointCloud2, PointField



@Processor("sensor_msgs/msg/PointCloud2", ["normalize_field"])
def normalize_field_range(
    msg,
    normalize_field: str = "intensity",
    field_vmin: float = 0.0,
    field_vmax: float = 100.0,
    target_vmin: float = 0.0,
    target_vmax: float = 1.0,
):
    """
    Normalize and clip arbitrary values in the point cloud.

    Args:
        msg: PointCloud2 message instance.
        normalize_field: PointCloud2 field name to normalize (e.g., intensity, x, y, z).
        field_vmin: Minimum source value used for normalization.
        field_vmax: Maximum source value used for normalization.
        target_vmin: Minimum output value to map normalized values to.
        target_vmax: Maximum output value to map normalized values to.

    Returns:
        PointCloud2: Message with arbitrary normalized values.
    """
    try:
        field_vmin, field_vmax = float(field_vmin), float(field_vmax)
        target_vmin, target_vmax = float(target_vmin), float(target_vmax)
    except (TypeError, ValueError):
        raise ValueError("field_vmin/field_vmax/target_vmin/target_vmax must be numeric")

    if field_vmax <= field_vmin:
        raise ValueError("field_vmax must be greater than field_vmin")
    if target_vmax <= target_vmin:
        raise ValueError("target_vmax must be greater than target_vmin")

    cloud_dtype = _build_cloud_dtype(msg)
    if normalize_field not in cloud_dtype.names:
        raise ValueError(f"Field '{normalize_field}' not found in message")

    data_array = bytearray(msg.data)
    
    cloud_view = np.ndarray(
        shape=(msg.height if msg.height > 0 else 1, msg.width),
        dtype=cloud_dtype,
        buffer=data_array,
        strides=(msg.row_step, msg.point_step),
    )

    target_field = cloud_view[normalize_field]
    
    # Clip to source range, normalize to [0, 1], then map to target range.
    normalized = np.clip((target_field - field_vmin) / (field_vmax - field_vmin), 0, 1)
    target_field[:] = target_vmin + normalized * (target_vmax - target_vmin)

    msg.data = bytes(data_array)
    
    return msg


@Processor("sensor_msgs/msg/PointCloud2", ["colormap"])
def apply_colormap(
    msg,
    normalize_field: str = "intensity",
    colormap_name: str = "summer",
    field_vmin: float = 0.0,
    field_vmax: float = 100.0
):
    """
    Adds an RGB field to the point cloud by normalizing the selected field
    and applying a colormap. Preserves all original fields.

    Args:
        msg: PointCloud2 message instance.
        normalize_field: PointCloud2 field name to use for colormap mapping (e.g., intensity, x, y, z).
        colormap_name: Name of the matplotlib colormap to use (e.g., "viridis", "plasma").
        field_vmin: This value will be mapped to the lowest color in the colormap. Values below this will be clipped.
        field_vmax: This value will be mapped to the highest color in the colormap. Values above this will be clipped.

    Returns:
        PointCloud2: Message with an added 'rgb' field containing the colormap-mapped colors based on the normalized field.
    """
    # 1. Validation and Setup
    try:
        normalize_field = str(normalize_field)
        field_vmin, field_vmax = float(field_vmin), float(field_vmax)
        colormap = plt.get_cmap(str(colormap_name))
    except Exception as e:
        raise ValueError(f"Invalid parameters: {e}")

    if field_vmax <= field_vmin:
        raise ValueError("field_vmax must be greater than field_vmin")

    cloud_dtype = _build_cloud_dtype(msg)
    if normalize_field not in cloud_dtype.names:
        raise ValueError(f"PointCloud2 missing required field: {normalize_field}")

    # 2. Map Buffer to Numpy
    data_array = bytearray(msg.data)
    cloud = np.ndarray(
        shape=(msg.height if msg.height > 0 else 1, msg.width),
        dtype=cloud_dtype,
        buffer=data_array,
        strides=(msg.row_step, msg.point_step),
    )

    # 3. Flatten and get all points
    cloud_flat = cloud.reshape(-1)
    
    # 4. Compute RGB colors
    normalizer = mcolors.Normalize(vmin=field_vmin, vmax=field_vmax, clip=True)
    vals_to_map = cloud_flat[normalize_field].astype(np.float32)
    rgba_colors = colormap(normalizer(vals_to_map))
    
    # 5. Pack RGB (ARGB format)
    rgb_255 = (rgba_colors[:, :3] * 255).astype(np.uint8)
    packed_rgb = (
        (np.uint32(255) << 24) | 
        (rgb_255[:, 0].astype(np.uint32) << 16) | 
        (rgb_255[:, 1].astype(np.uint32) << 8) | 
        rgb_255[:, 2].astype(np.uint32)
    )

    # 6. Build Output Buffer with original fields + rgb
    endian_char = ">" if msg.is_bigendian else "<"
    output_dtype = cloud_dtype.descr
    output_dtype.append(("rgb", f"{endian_char}u4"))
    output_dtype = np.dtype(output_dtype)
    
    pcd_out = np.zeros(len(cloud_flat), dtype=output_dtype)
    
    # Copy all original fields in one vectorized operation
    for field_name in cloud_dtype.names:
        pcd_out[field_name] = cloud_flat[field_name]
    pcd_out["rgb"] = packed_rgb

    # 7. Construct Result Message
    out = PointCloud2()
    out.header = msg.header
    out.height, out.width = 1, len(pcd_out)
    out.is_bigendian, out.point_step, out.is_dense = msg.is_bigendian, output_dtype.itemsize, msg.is_dense
    
    # Build fields list: original fields + rgb
    fields = list(msg.fields)
    rgb_field = PointField(name="rgb", offset=output_dtype.fields["rgb"][1], datatype=PointField.UINT32, count=1)
    fields.append(rgb_field)
    out.fields = fields
    
    out.row_step = out.point_step * out.width
    out.data = pcd_out.tobytes()

    return out


def _build_cloud_dtype(msg: PointCloud2) -> np.dtype:
	field_types = {
		PointField.INT8: np.int8,
		PointField.UINT8: np.uint8,
		PointField.INT16: np.int16,
		PointField.UINT16: np.uint16,
		PointField.INT32: np.int32,
		PointField.UINT32: np.uint32,
		PointField.FLOAT32: np.float32,
		PointField.FLOAT64: np.float64,
	}

	field_names = []
	field_formats = []
	field_offsets = []
	for field in msg.fields:
		base_type = field_types.get(field.datatype)
		if base_type is None:
			raise ValueError(f"Unsupported PointField datatype: {field.datatype}")
		count = field.count if hasattr(field, "count") and field.count else 1
		fmt = (base_type, int(count)) if count > 1 else base_type
		field_names.append(field.name)
		field_formats.append(np.dtype(fmt))
		field_offsets.append(field.offset)

	cloud_dtype = np.dtype(
		{
			"names": field_names,
			"formats": field_formats,
			"offsets": field_offsets,
			"itemsize": msg.point_step,
		}
	)
	return cloud_dtype.newbyteorder(">" if msg.is_bigendian else "<")
