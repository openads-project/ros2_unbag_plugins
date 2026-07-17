## Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
## SPDX-License-Identifier: Apache-2.0

from ros2_unbag.core.processors.base import Processor
from ros2_unbag.core.processors import pointcloud as _pointcloud

try:
    from ros2_unbag.core.processors import colormap as _colormap
except (ImportError, ModuleNotFoundError):
    try:
        from custom.processors import colormap as _colormap
    except (ImportError, ModuleNotFoundError):
        # Optional colormap processor not available — provide a clear fallback that will raise only if used.
        class _colormap:
            @staticmethod
            def normalize_field_range(*args, **kwargs):
                raise RuntimeError(
                    "Optional processor 'normalize_field_range' is not available. "
                    "Install the optional dependency providing 'custom.processors.colormap' "
                    "or avoid using normalization features."
                )

            @staticmethod
            def apply_colormap(*args, **kwargs):
                raise RuntimeError(
                    "Optional processor 'apply_colormap' is not available. "
                    "Install the optional dependency providing 'custom.processors.colormap' "
                    "or avoid using colormap features."
                )

try:
    from ros2_unbag.core.routines.cloudini_pointcloud import decode_cloudini_compressed_pointcloud
except (ImportError, ModuleNotFoundError):
    # Optional routine not available — provide a clear fallback that will raise only if used.
    def decode_cloudini_compressed_pointcloud(msg):
        raise RuntimeError(
            "Optional routine 'decode_cloudini_compressed_pointcloud' is not available. "
            "Install the optional dependency providing 'ros2_unbag.core.routines.cloudini_pointcloud' "
            "or avoid processing compressed Cloudini pointcloud messages."
        )

# The base pointcloud processors are decorated with Processor, so we need the wrapped
# handler functions rather than the decorator instances themselves.
_pc_field_mapping = getattr(_pointcloud.pointcloud_apply_field_mapping, "func", _pointcloud.pointcloud_apply_field_mapping)
_pc_remove_fields = getattr(_pointcloud.pointcloud_remove_fields, "func", _pointcloud.pointcloud_remove_fields)
_pc_transform_from_yaml = getattr(_pointcloud.pointcloud_apply_transform_from_yaml, "func", _pointcloud.pointcloud_apply_transform_from_yaml)
_pc_transform = getattr(_pointcloud.pointcloud_apply_transform, "func", _pointcloud.pointcloud_apply_transform)

_pc_normalize_field_range = getattr(_colormap.normalize_field_range, "func", _colormap.normalize_field_range)
_pc_apply_colormap = getattr(_colormap.apply_colormap, "func", _colormap.apply_colormap)


@Processor("point_cloud_interfaces/msg/CompressedPointCloud2", ["field_mapping"])
def cloudini_pointcloud_apply_field_mapping(msg, field_mapping: str):
    """
    Apply a field mapping to a PointCloud2 message.

    Args:
        msg: PointCloud2 message instance.
        field_mapping: "field_name: new_field_name, field_name2: new_field_name2, ..."

    Returns:
        PointCloud2: Modified PointCloud2 message with remapped fields.

    Raises:
        ValueError: If field_mapping is invalid or fields do not exist in the message.
    """
    msg = decode_cloudini_compressed_pointcloud(msg)
    return _pc_field_mapping(msg, field_mapping)
   
   

@Processor("point_cloud_interfaces/msg/CompressedPointCloud2", ["remove_fields"])
def cloudini_pointcloud_remove_fields(msg, fields_to_remove: str):
    """
    Remove specified fields from a PointCloud2 message.

    Args:
        msg: PointCloud2 message instance.
        fields_to_remove: "field_name, field_name2, ..."

    Returns:
        PointCloud2: Modified PointCloud2 message with specified fields removed.
    """
    msg = decode_cloudini_compressed_pointcloud(msg)
    return _pc_remove_fields(msg, fields_to_remove)
   

@Processor("point_cloud_interfaces/msg/CompressedPointCloud2", ["transform_from_yaml"])
def cloudini_pointcloud_apply_transform_from_yaml(msg, custom_frame_path: str):
    """
    Apply a rigid-body transform from a YAML file to all points in a PointCloud2 message.

    Args:
        msg: PointCloud2 message instance.
        custom_frame_path: Path to YAML file containing translation as x, y, z and rotation as x, y, z, w.

    Returns:
        PointCloud2: Transformed PointCloud2 message.

    Raises:
        ValueError: If file path is invalid or message fields are missing.
    """
    msg = decode_cloudini_compressed_pointcloud(msg)
    return _pc_transform_from_yaml(msg, custom_frame_path)


@Processor("point_cloud_interfaces/msg/CompressedPointCloud2", ["transform"])
def cloudini_pointcloud_apply_transform(msg, translation_x: float, translation_y: float, translation_z: float, rotation_x: float, rotation_y: float, rotation_z: float, rotation_w: float):
    """
    Apply a rigid-body transform directly from translation and quaternion components.

    Args:
        msg: PointCloud2 message instance.
        translation_x: Translation along x-axis.
        translation_y: Translation along y-axis.
        translation_z: Translation along z-axis.
        rotation_x: Quaternion x component.
        rotation_y: Quaternion y component.
        rotation_z: Quaternion z component.
        rotation_w: Quaternion w component.

    Returns:
        PointCloud2: Transformed PointCloud2 message.

    Raises:
        ValueError: If inputs are not numeric or message fields are missing.
    """
    msg = decode_cloudini_compressed_pointcloud(msg)
    return _pc_transform(msg, translation_x, translation_y, translation_z, rotation_x, rotation_y, rotation_z, rotation_w)


@Processor("point_cloud_interfaces/msg/CompressedPointCloud2", ["normalize_field"])
def cloudini_pointcloud_normalize_field_range(
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
    msg = decode_cloudini_compressed_pointcloud(msg)
    return _pc_normalize_field_range(msg, normalize_field, field_vmin, field_vmax, target_vmin, target_vmax)


@Processor("point_cloud_interfaces/msg/CompressedPointCloud2", ["colormap"])
def cloudini_apply_colormap(
    msg,
    normalize_field: str = "intensity",
    colormap_name: str = "summer",
    field_vmin: float = 0.0,
    field_vmax: float = 100.0,
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
    msg = decode_cloudini_compressed_pointcloud(msg)
    return _pc_apply_colormap(msg, normalize_field, colormap_name, field_vmin, field_vmax)
