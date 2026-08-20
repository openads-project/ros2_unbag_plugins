# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

from pathlib import Path

from ros2_unbag.core.routines.base import ExportMetadata, ExportMode, ExportRoutine
from ros2_unbag.core.routines.pointcloud import (
    export_pointcloud_pcd as _export_pointcloud_pcd,
    export_pointcloud_pkl as _export_pointcloud_pkl,
    export_pointcloud_xyz as _export_pointcloud_xyz,
)

import ctypes
import ctypes.util
import math
from pathlib import Path
import struct
from dataclasses import dataclass, field
from enum import IntEnum
from typing import List, Optional

import yaml
from sensor_msgs.msg import PointCloud2, PointField

def _decode(msg):
    return decode_cloudini_compressed_pointcloud(msg)


@ExportRoutine(
    "point_cloud_interfaces/msg/CompressedPointCloud2",
    ["pointcloud/pkl"],
    mode=ExportMode.MULTI_FILE,
)
def export_cloudini_pointcloud_pkl(msg, path: Path, fmt: str, metadata: ExportMetadata):
    decompressed = _decode(msg)
    _export_pointcloud_pkl(decompressed, path, fmt, metadata)


@ExportRoutine(
    "point_cloud_interfaces/msg/CompressedPointCloud2",
    ["pointcloud/xyz"],
    mode=ExportMode.MULTI_FILE,
)
def export_cloudini_pointcloud_xyz(msg, path: Path, fmt: str, metadata: ExportMetadata):
    decompressed = _decode(msg)
    _export_pointcloud_xyz(decompressed, path, fmt, metadata)


@ExportRoutine(
    "point_cloud_interfaces/msg/CompressedPointCloud2",
    ["pointcloud/pcd", "pointcloud/pcd_compressed", "pointcloud/pcd_ascii"],
    mode=ExportMode.MULTI_FILE,
)
def export_cloudini_pointcloud_pcd(msg, path: Path, fmt: str, metadata: ExportMetadata):
    decompressed = _decode(msg)
    _export_pointcloud_pcd(decompressed, path, fmt, metadata)

###################### UTILS ####################
#################################################
__all__ = ["decode_cloudini_compressed_pointcloud"]

_K_MAGIC_HEADER = b"CLOUDINI_V"
_K_MAGIC_LENGTH = len(_K_MAGIC_HEADER)
_K_DECODE_SKIP = 0xFFFFFFFF
_K_MAX_VERSION = 3

_PF_INT8 = getattr(PointField, "INT8", 1)
_PF_UINT8 = getattr(PointField, "UINT8", 2)
_PF_INT16 = getattr(PointField, "INT16", 3)
_PF_UINT16 = getattr(PointField, "UINT16", 4)
_PF_INT32 = getattr(PointField, "INT32", 5)
_PF_UINT32 = getattr(PointField, "UINT32", 6)
_PF_FLOAT32 = getattr(PointField, "FLOAT32", 7)
_PF_FLOAT64 = getattr(PointField, "FLOAT64", 8)
_PF_INT64 = getattr(PointField, "INT64", 9)
_PF_UINT64 = getattr(PointField, "UINT64", 10)


class EncodingOptions(IntEnum):
    NONE = 0
    LOSSY = 1
    LOSSLESS = 2


class CompressionOption(IntEnum):
    NONE = 0
    LZ4 = 1
    ZSTD = 2


@dataclass
class CloudiniField:
    name: str
    offset: int
    type: int
    resolution: Optional[float]


@dataclass
class EncodingInfo:
    fields: List[CloudiniField] = field(default_factory=list)
    width: int = 0
    height: int = 1
    point_step: int = 0
    encoding_opt: EncodingOptions = EncodingOptions.LOSSY
    compression_opt: CompressionOption = CompressionOption.ZSTD
    version: int = _K_MAX_VERSION


class _ByteReader:
    __slots__ = ("_buffer", "_offset")

    def __init__(self, data: bytes):
        self._buffer = data
        self._offset = 0

    def read_bytes(self, size: int) -> bytes:
        end = self._offset + size
        if size < 0 or end > len(self._buffer):
            raise ValueError("Buffer underrun while decoding Cloudini payload")
        chunk = self._buffer[self._offset:end]
        self._offset = end
        return chunk

    def read_byte(self) -> int:
        return self._read_int(1)

    def read_uint16(self) -> int:
        return self._read_int(2)

    def read_uint32(self) -> int:
        return self._read_int(4)

    def read_string(self) -> str:
        size = self.read_uint16()
        data = self.read_bytes(size)
        return data.decode("utf-8")

    def read_cstring(self) -> bytes:
        start = self._offset
        end = self._buffer.find(b"\0", start)
        if end == -1:
            raise ValueError("Cloudini header missing null terminator")
        self._offset = end + 1
        return self._buffer[start:end]

    def peek_bytes(self, size: int) -> bytes:
        end = self._offset + size
        if size < 0 or end > len(self._buffer):
            raise ValueError("Buffer underrun while peeking Cloudini payload")
        return self._buffer[self._offset:end]

    def remaining(self) -> int:
        return len(self._buffer) - self._offset

    def _read_int(self, size: int) -> int:
        data = self.read_bytes(size)
        return int.from_bytes(data, "little", signed=False)


def decode_cloudini_compressed_pointcloud(msg) -> PointCloud2:
    # Allow already-decoded PointCloud2 objects to pass through so processors
    # can be chained without re-encoding.
    if isinstance(msg, PointCloud2):
        return msg

    if getattr(msg, "format", "cloudini") not in ("cloudini", ""):
        raise ValueError(f"Unsupported compressed point cloud format: {msg.format}")

    if not hasattr(msg, "compressed_data"):
        raise AttributeError("Message does not contain 'compressed_data'; expected Cloudini-compressed input")

    compressed_bytes = bytes(msg.compressed_data)
    reader = _ByteReader(compressed_bytes)
    encoding_info = _decode_header(reader)

    total_points = encoding_info.width * encoding_info.height
    expected_size = total_points * encoding_info.point_step

    raw_data = _decode_payload(encoding_info, reader, expected_size)

    if expected_size and len(raw_data) != expected_size:
        raise ValueError(
            "Decoded point cloud size mismatch (expected "
            f"{expected_size} bytes, got {len(raw_data)})"
        )

    pointcloud = PointCloud2()
    pointcloud.header = msg.header
    pointcloud.height = encoding_info.height
    pointcloud.width = encoding_info.width
    pointcloud.fields = _resolve_point_fields(list(getattr(msg, "fields", [])), encoding_info.fields)
    pointcloud.is_bigendian = bool(getattr(msg, "is_bigendian", False))
    pointcloud.point_step = encoding_info.point_step
    pointcloud.row_step = encoding_info.point_step * encoding_info.width
    pointcloud.is_dense = bool(msg.is_dense)
    pointcloud.data = raw_data
    return pointcloud


def _decode_header(reader: _ByteReader) -> EncodingInfo:
    if reader.remaining() < _K_MAGIC_LENGTH + 2:
        raise ValueError("Compressed point cloud data too short to contain Cloudini header")

    magic = reader.read_bytes(_K_MAGIC_LENGTH)
    if magic != _K_MAGIC_HEADER:
        raise ValueError("Invalid Cloudini magic header")

    version_digits = reader.read_bytes(2)
    version = (version_digits[0] - 48) * 10 + (version_digits[1] - 48)
    if version < 2 or version > _K_MAX_VERSION:
        raise ValueError(f"Unsupported Cloudini encoding version: {version}")

    info = EncodingInfo()
    info.version = version

    if reader.remaining() > 1:
        peek = reader.peek_bytes(2)
        if peek[0] == 0x0A and peek[1] != ord("{"):
            saved_offset = reader._offset
            try:
                reader.read_bytes(1)
                yaml_blob = reader.read_cstring()
                info = _encoding_info_from_yaml(yaml_blob.decode("utf-8"))
                info.version = version
                return info
            except (ValueError, UnicodeDecodeError, yaml.YAMLError):
                reader._offset = saved_offset

    info.width = reader.read_uint32()
    info.height = reader.read_uint32()
    info.point_step = reader.read_uint32()

    info.encoding_opt = EncodingOptions(reader.read_byte())
    info.compression_opt = CompressionOption(reader.read_byte())

    fields_count = reader.read_uint16()
    info.fields = []
    for _ in range(fields_count):
        name = reader.read_string()
        offset = reader.read_uint32()
        field_type = int(reader.read_byte())
        resolution_bytes = reader.read_bytes(4)
        resolution_value = struct.unpack("<f", resolution_bytes)[0]
        resolution = resolution_value if resolution_value > 0 else None
        info.fields.append(CloudiniField(name=name, offset=offset, type=field_type, resolution=resolution))

    return info


def _encoding_info_from_yaml(yaml_text: str) -> EncodingInfo:
    data = yaml.safe_load(yaml_text) or {}

    info = EncodingInfo()
    info.version = int(data.get("version", _K_MAX_VERSION))
    info.width = int(data.get("width", 0))
    info.height = int(data.get("height", 1))
    info.point_step = int(data.get("point_step", 0))
    info.encoding_opt = _encoding_option_from_value(data.get("encoding_opt", EncodingOptions.LOSSY))
    info.compression_opt = _compression_option_from_value(data.get("compression_opt", CompressionOption.ZSTD))

    fields = []
    for entry in data.get("fields", []):
        name = str(entry.get("name", ""))
        offset = int(entry.get("offset", 0))
        field_type = _field_type_from_value(entry.get("type", 0))
        resolution_value = entry.get("resolution", None)
        resolution = float(resolution_value) if resolution_value not in (None, "null") else None
        fields.append(CloudiniField(name=name, offset=offset, type=field_type, resolution=resolution))

    info.fields = fields
    return info


def _encoding_option_from_value(value) -> EncodingOptions:
    if isinstance(value, EncodingOptions):
        return value
    if isinstance(value, str):
        value = value.strip().upper()
        if value in ("NONE", "LOSSY", "LOSSLESS"):
            return EncodingOptions[value]
        value = int(value)
    return EncodingOptions(int(value))


def _compression_option_from_value(value) -> CompressionOption:
    if isinstance(value, CompressionOption):
        return value
    if isinstance(value, str):
        value = value.strip().upper()
        if value in ("NONE", "LZ4", "ZSTD"):
            return CompressionOption[value]
        value = int(value)
    return CompressionOption(int(value))


def _field_type_from_value(value) -> int:
    if isinstance(value, int):
        return value
    cleaned = str(value).strip().upper()
    mapping = {
        "INT8": _PF_INT8,
        "UINT8": _PF_UINT8,
        "INT16": _PF_INT16,
        "UINT16": _PF_UINT16,
        "INT32": _PF_INT32,
        "UINT32": _PF_UINT32,
        "FLOAT32": _PF_FLOAT32,
        "FLOAT64": _PF_FLOAT64,
        "INT64": _PF_INT64,
        "UINT64": _PF_UINT64,
    }
    if cleaned in mapping:
        return mapping[cleaned]
    return int(cleaned)


def _decode_payload(info: EncodingInfo, reader: _ByteReader, total_capacity: int) -> bytes:
    if total_capacity == 0:
        return b""

    decoders = _build_decoders(info)
    output = bytearray(total_capacity)
    written = 0

    if info.version >= 3:
        while reader.remaining() > 0:
            chunk_size = reader.read_uint32()
            if chunk_size > reader.remaining():
                raise ValueError("Invalid Cloudini chunk size")
            chunk = reader.read_bytes(chunk_size)
            written += _decode_chunk(info, decoders, chunk, output, written)
    else:
        chunk = reader.read_bytes(reader.remaining())
        written += _decode_chunk(info, decoders, chunk, output, written)

    if written < total_capacity:
        output = output[:written]
    elif written > total_capacity:
        raise ValueError("Decoded point cloud larger than expected capacity")

    return bytes(output)


def _decode_chunk(info: EncodingInfo, decoders, chunk: bytes, output: bytearray, base_offset: int) -> int:
    if not chunk:
        return 0

    remaining_capacity = len(output) - base_offset
    if remaining_capacity <= 0:
        raise ValueError("No space left while decoding Cloudini chunk")

    if info.compression_opt == CompressionOption.NONE:
        encoded = chunk
    elif info.compression_opt == CompressionOption.LZ4:
        encoded = _decompress_lz4(chunk, remaining_capacity)
    elif info.compression_opt == CompressionOption.ZSTD:
        encoded = _decompress_zstd(chunk, remaining_capacity)
    else:
        raise ValueError(f"Unsupported Cloudini compression option: {info.compression_opt}")

    encoded_reader = _ByteReader(encoded)
    for decoder in decoders:
        decoder.reset()

    written = 0
    point_step = info.point_step
    while encoded_reader.remaining() > 0:
        point_base = base_offset + written
        if point_base + point_step > len(output):
            raise ValueError("Decoded Cloudini data exceeds preallocated buffer")
        for decoder in decoders:
            decoder.decode(encoded_reader, output, point_base)
        written += point_step

    return written


def _resolve_point_fields(msg_fields, info_fields):
    if msg_fields and len(msg_fields) == len(info_fields):
        matches = True
        for msg_field, info_field in zip(msg_fields, info_fields):
            if (
                msg_field.name != info_field.name
                or msg_field.offset != info_field.offset
                or getattr(msg_field, "datatype", None) != info_field.type
            ):
                matches = False
                break
        if matches:
            return msg_fields

    fields = []
    for info_field in info_fields:
        pf = PointField()
        pf.name = info_field.name
        pf.offset = info_field.offset
        pf.datatype = info_field.type
        pf.count = 1
        fields.append(pf)
    return fields


class _FieldDecoder:
    def decode(self, reader: _ByteReader, dest: bytearray, base: int) -> None:
        raise NotImplementedError

    def reset(self) -> None:
        pass


class _FieldDecoderCopy(_FieldDecoder):
    def __init__(self, offset: int, size: int):
        self._offset = None if offset == _K_DECODE_SKIP else offset
        self._size = size

    def decode(self, reader: _ByteReader, dest: bytearray, base: int) -> None:
        data = reader.read_bytes(self._size)
        if self._offset is not None:
            start = base + self._offset
            dest[start:start + self._size] = data


class _FieldDecoderInt(_FieldDecoder):
    def __init__(self, offset: int, size: int, signed: bool):
        self._offset = None if offset == _K_DECODE_SKIP else offset
        self._size = size
        self._signed = signed
        self._prev_value = 0

    def decode(self, reader: _ByteReader, dest: bytearray, base: int) -> None:
        diff = _read_varint(reader)
        value = self._prev_value + diff
        self._prev_value = value
        if self._offset is None:
            return
        start = base + self._offset
        if self._signed:
            packed = int(value).to_bytes(self._size, "little", signed=True)
        else:
            mask = (1 << (self._size * 8)) - 1
            packed = int(value & mask).to_bytes(self._size, "little", signed=False)
        dest[start:start + self._size] = packed

    def reset(self) -> None:
        self._prev_value = 0


class _FieldDecoderFloatLossy(_FieldDecoder):
    def __init__(self, offset: int, resolution: float, fmt: str):
        if resolution <= 0.0:
            raise ValueError("Cloudini lossy decoder requires resolution > 0")
        self._offset = None if offset == _K_DECODE_SKIP else offset
        self._resolution = resolution
        self._fmt = fmt
        self._prev_value = 0

    def decode(self, reader: _ByteReader, dest: bytearray, base: int) -> None:
        if reader.peek_bytes(1)[0] == 0:
            reader.read_byte()
            self._prev_value = 0
            if self._offset is not None:
                start = base + self._offset
                dest[start:start + struct.calcsize(self._fmt)] = struct.pack(self._fmt, math.nan)
            return

        diff = _read_varint(reader)
        current = self._prev_value + diff
        self._prev_value = current
        if self._offset is not None:
            value = current * self._resolution
            start = base + self._offset
            dest[start:start + struct.calcsize(self._fmt)] = struct.pack(self._fmt, value)

    def reset(self) -> None:
        self._prev_value = 0


class _FieldDecoderFloatXor(_FieldDecoder):
    def __init__(self, offset: int, size: int):
        self._offset = None if offset == _K_DECODE_SKIP else offset
        self._size = size
        self._prev_bits = 0

    def decode(self, reader: _ByteReader, dest: bytearray, base: int) -> None:
        residual = int.from_bytes(reader.read_bytes(self._size), "little", signed=False)
        current = residual ^ self._prev_bits
        self._prev_bits = current
        if self._offset is not None:
            start = base + self._offset
            dest[start:start + self._size] = current.to_bytes(self._size, "little", signed=False)

    def reset(self) -> None:
        self._prev_bits = 0


class _FieldDecoderFloatNLossy(_FieldDecoder):
    def __init__(self, offsets, resolutions):
        self._offsets = [None if off == _K_DECODE_SKIP else off for off in offsets]
        self._res = resolutions
        self._count = len(offsets)
        self._prev = [0, 0, 0, 0]

    def decode(self, reader: _ByteReader, dest: bytearray, base: int) -> None:
        for index in range(self._count):
            if reader.peek_bytes(1)[0] == 0:
                reader.read_byte()
                self._prev[index] = 0
                value = math.nan
            else:
                diff = _read_varint(reader)
                current = self._prev[index] + diff
                self._prev[index] = current
                value = current * self._res[index]
            offset = self._offsets[index]
            if offset is not None:
                start = base + offset
                dest[start:start + 4] = struct.pack("<f", value)

    def reset(self) -> None:
        for i in range(self._count):
            self._prev[i] = 0


def _build_decoders(info: EncodingInfo):
    decoders: List[_FieldDecoder] = []

    if info.encoding_opt == EncodingOptions.NONE:
        for field in info.fields:
            decoders.append(_FieldDecoderCopy(field.offset, _field_size(field.type)))
        return decoders

    start_index = 0
    if info.encoding_opt == EncodingOptions.LOSSY:
        floats_count = 0
        for field in info.fields:
            if field.type != _PF_FLOAT32 or field.resolution is None:
                break
            floats_count += 1
        if floats_count in (3, 4):
            offsets = [info.fields[i].offset for i in range(floats_count)]
            resolutions = [float(info.fields[i].resolution) for i in range(floats_count)]
            decoders.append(_FieldDecoderFloatNLossy(offsets, resolutions))
            start_index = floats_count

    for field in info.fields[start_index:]:
        decoders.append(_make_decoder(field, info.encoding_opt))

    return decoders


def _make_decoder(field: CloudiniField, encoding_opt: EncodingOptions) -> _FieldDecoder:
    offset = field.offset
    if field.type == _PF_FLOAT32:
        if encoding_opt == EncodingOptions.LOSSY and field.resolution is not None:
            return _FieldDecoderFloatLossy(offset, float(field.resolution), "<f")
        return _FieldDecoderCopy(offset, 4)
    if field.type == _PF_FLOAT64:
        if encoding_opt == EncodingOptions.LOSSY and field.resolution is not None:
            return _FieldDecoderFloatLossy(offset, float(field.resolution), "<d")
        return _FieldDecoderFloatXor(offset, 8)
    if field.type == _PF_INT16:
        return _FieldDecoderInt(offset, 2, True)
    if field.type == _PF_INT32:
        return _FieldDecoderInt(offset, 4, True)
    if field.type == _PF_UINT16:
        return _FieldDecoderInt(offset, 2, False)
    if field.type == _PF_UINT32:
        return _FieldDecoderInt(offset, 4, False)
    if field.type in (_PF_UINT64, 10):
        return _FieldDecoderInt(offset, 8, False)
    if field.type in (_PF_INT64, 9):
        return _FieldDecoderInt(offset, 8, True)
    if field.type in (_PF_INT8, _PF_UINT8):
        return _FieldDecoderCopy(offset, 1)
    raise ValueError(f"Unsupported Cloudini field type: {field.type}")


def _field_size(field_type: int) -> int:
    if field_type in (_PF_INT8, _PF_UINT8):
        return 1
    if field_type in (_PF_INT16, _PF_UINT16):
        return 2
    if field_type in (_PF_INT32, _PF_UINT32, _PF_FLOAT32):
        return 4
    if field_type in (_PF_INT64, _PF_UINT64, _PF_FLOAT64, 9, 10):
        return 8
    return 0


def _read_varint(reader: _ByteReader) -> int:
    uval = 0
    shift = 0
    while True:
        byte = reader.read_byte()
        uval |= (byte & 0x7F) << shift
        if (byte & 0x80) == 0:
            break
        shift += 7
    uval -= 1
    return (uval >> 1) ^ -(uval & 1)


_LZ4_LIB = None
_LZ4_DECOMPRESS = None


def _ensure_lz4():
    global _LZ4_LIB, _LZ4_DECOMPRESS
    if _LZ4_LIB is not None:
        return
    lib_path = ctypes.util.find_library("lz4")
    if not lib_path:
        raise RuntimeError("liblz4 not found on the system")
    _LZ4_LIB = ctypes.cdll.LoadLibrary(lib_path)
    _LZ4_DECOMPRESS = _LZ4_LIB.LZ4_decompress_safe
    _LZ4_DECOMPRESS.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
    _LZ4_DECOMPRESS.restype = ctypes.c_int


def _decompress_lz4(data: bytes, capacity: int) -> bytes:
    _ensure_lz4()
    if capacity <= 0:
        return b""
    src = (ctypes.c_char * len(data)).from_buffer_copy(data)
    dst = (ctypes.c_char * capacity)()
    result = _LZ4_DECOMPRESS(src, dst, len(data), capacity)
    if result < 0:
        raise RuntimeError("LZ4 decompression failed")
    return bytes(dst[:result])


_ZSTD_LIB = None
_ZSTD_DECOMPRESS = None
_ZSTD_IS_ERROR = None
_ZSTD_ERROR_NAME = None


def _ensure_zstd():
    global _ZSTD_LIB, _ZSTD_DECOMPRESS, _ZSTD_IS_ERROR, _ZSTD_ERROR_NAME
    if _ZSTD_LIB is not None:
        return
    lib_path = ctypes.util.find_library("zstd")
    if not lib_path:
        raise RuntimeError("libzstd not found on the system")
    _ZSTD_LIB = ctypes.cdll.LoadLibrary(lib_path)
    _ZSTD_DECOMPRESS = _ZSTD_LIB.ZSTD_decompress
    _ZSTD_DECOMPRESS.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p, ctypes.c_size_t]
    _ZSTD_DECOMPRESS.restype = ctypes.c_size_t
    _ZSTD_IS_ERROR = _ZSTD_LIB.ZSTD_isError
    _ZSTD_IS_ERROR.argtypes = [ctypes.c_size_t]
    _ZSTD_IS_ERROR.restype = ctypes.c_uint
    _ZSTD_ERROR_NAME = _ZSTD_LIB.ZSTD_getErrorName
    _ZSTD_ERROR_NAME.argtypes = [ctypes.c_size_t]
    _ZSTD_ERROR_NAME.restype = ctypes.c_char_p


def _decompress_zstd(data: bytes, capacity: int) -> bytes:
    _ensure_zstd()
    if capacity <= 0:
        return b""
    src = (ctypes.c_char * len(data)).from_buffer_copy(data)
    dst = (ctypes.c_char * capacity)()
    result = _ZSTD_DECOMPRESS(dst, capacity, src, len(data))
    if _ZSTD_IS_ERROR(result):
        err = _ZSTD_ERROR_NAME(result)
        message = err.decode("utf-8") if err else "unknown"
        raise RuntimeError(f"ZSTD decompression failed: {message}")
    return bytes(dst[:result])
