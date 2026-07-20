# Implementation Details

## Routines

| Routine | Message Type | Format | Description |
| --- | --- | --- | --- |
| [`cloudini_pointcloud`](../plugins/routines/cloudini_pointcloud.py) | `sensor_msgs/msg/PointCloud2` | `.pcd`, `.pkl`, `.xyz` | Exports point clouds compressed with [Cloudini](https://github.com/facontidavide/cloudini) |
| [`object_list`](../plugins/routines/object_list.py) | `perception_msgs/msg/ObjectList` | `.csv` (HEXAMOTION) | Exports object lists in the HEXAMOTION format |

## Processors

| Processor | Message Type | Description |
| --- | --- | --- |
| [`cloudini_pointcloud`](../plugins/processors/cloudini_pointcloud.py) | `sensor_msgs/msg/PointCloud2` | Decompresses Cloudini-compressed point cloud fields before export |
| [`colormap`](../plugins/processors/colormap.py) | `sensor_msgs/msg/PointCloud2` | Normalizes a point field into a target range, e.g., for intensity colorization |
| [`object_list`](../plugins/processors/object_list.py) | `perception_msgs/msg/ObjectList` | Applies a static transform to all objects in the list |

Routines and processors are installed into the `ros2_unbag` CLI/GUI via `ros2 unbag --install-routine` / `ros2 unbag --install-processor` during the Docker image build, see [`docker/custom.sh`](../docker/custom.sh).
