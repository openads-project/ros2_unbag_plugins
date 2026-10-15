# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

import struct

import pytest
from point_cloud_interfaces.msg import CompressedPointCloud2
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2, PointField

from _cloudini_test_encoder import encode
from ros2_unbag.core.routines.cloudini_pointcloud import decode_cloudini_compressed_pointcloud as decode


def make_cloud(height=2, width=3):
    cloud = PointCloud2(height=height, width=width, point_step=16, row_step=width * 16, is_dense=False)
    cloud.header.frame_id = 'lidar'
    cloud.header.stamp.sec = 123
    cloud.header.stamp.nanosec = 456
    cloud.fields = [
        PointField(name=name, offset=offset, datatype=datatype, count=1)
        for name, offset, datatype in [
            ('x', 0, PointField.FLOAT32), ('y', 4, PointField.FLOAT32),
            ('z', 8, PointField.FLOAT32), ('intensity', 12, PointField.UINT32),
        ]
    ]
    cloud.data = b''.join(
        struct.pack('<fffI', i + 0.1234, -i - 0.4567, i * 0.1, 2**31 + i)
        for i in range(height * width)
    )
    return cloud


def compress(cloud, version=5, lossy=False):
    # Upstream's current encoder always emits chunk framing, including when
    # asked for v2. Build a valid legacy v2 fixture from the equivalent single
    # v3 chunk: v2 uses the same codec without the chunk-size prefix.
    msg = deserialize_message(encode(serialize_message(cloud), max(3, version), lossy), CompressedPointCloud2)
    if version == 2 and cloud.width * cloud.height:
        payload = bytes(msg.compressed_data)
        assert payload[12:13] == b'\n'  # upstream's default YAML header
        header_end = payload.index(b'\0', 12) + 1
        assert struct.unpack_from('<I', payload, header_end)[0] == len(payload) - header_end - 4
        msg.compressed_data = payload[:10] + b'02' + payload[12:header_end] + payload[header_end + 4:]
    if cloud.width * cloud.height:
        assert bytes(msg.compressed_data).startswith(f'CLOUDINI_V{version:02d}'.encode())
    return msg


@pytest.mark.parametrize('version', [2, 3, 4, 5, 6])
@pytest.mark.parametrize('lossy', [False, True])
def test_upstream_round_trip(version, lossy):
    cloud = make_cloud()
    decoded = decode(compress(cloud, version, lossy))
    assert decoded.header == cloud.header
    assert decoded.fields == cloud.fields
    assert (decoded.height, decoded.width) == (cloud.height, cloud.width)
    assert (decoded.point_step, decoded.row_step) == (cloud.point_step, cloud.row_step)
    assert decoded.is_dense == cloud.is_dense
    assert decoded.is_bigendian == cloud.is_bigendian
    if not lossy:
        assert bytes(decoded.data) == bytes(cloud.data)
    else:
        original = list(struct.iter_unpack('<fffI', cloud.data))
        restored = list(struct.iter_unpack('<fffI', decoded.data))
        for expected, actual in zip(original, restored):
            assert actual[:3] == pytest.approx(expected[:3], abs=0.000501)
            assert actual[3] == expected[3]


def test_empty_cloud():
    cloud = make_cloud(height=1, width=0)
    decoded = decode(compress(cloud))
    assert decoded.width == 0
    assert not decoded.data
    assert decoded.header == cloud.header


def test_processor_chaining():
    cloud = make_cloud()
    assert decode(cloud) is cloud
    from ros2_unbag.core.processors.cloudini_pointcloud import cloudini_pointcloud_remove_fields
    processor = getattr(cloudini_pointcloud_remove_fields, 'func', cloudini_pointcloud_remove_fields)
    decoded = processor(compress(cloud), 'intensity')
    assert [field.name for field in decoded.fields] == ['x', 'y', 'z']


@pytest.mark.parametrize('bad_version', [1, 99])
def test_unsupported_version(bad_version):
    msg = compress(make_cloud())
    payload = bytearray(msg.compressed_data)
    payload[10:12] = f'{bad_version:02d}'.encode()
    msg.compressed_data = bytes(payload)
    with pytest.raises((RuntimeError, ValueError)):
        decode(msg)


def test_invalid_payload():
    msg = compress(make_cloud())
    msg.compressed_data = b'not a cloudini payload'
    with pytest.raises((RuntimeError, ValueError)):
        decode(msg)


def test_wrong_format():
    msg = compress(make_cloud())
    msg.format = 'other'
    with pytest.raises(ValueError, match='Unsupported compressed point cloud format'):
        decode(msg)


def test_missing_payload():
    with pytest.raises(AttributeError, match='compressed_data'):
        decode(object())
