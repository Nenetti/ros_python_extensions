from __future__ import annotations

import numpy as np
import sensor_msgs.msg as sensor_msgs

from ._point_type import PointType

__all__ = [
    "pointcloud2_to_numpy_dict",
]


# ======================================================================================================================
#
#   Public Methods
#
# ======================================================================================================================
def pointcloud2_to_numpy_dict(msg: sensor_msgs.PointCloud2) -> dict[str, np.ndarray]:
    """
    Converts PointCloud2 msg to dict which has numpy.

    Args:
        msg (sensor_msgs.PointCloud2):

    Returns:
        {"xyz": [H, W, 3], "rgb": [H, W, 3], "rgba": [H, W, 4], ...}.
    """
    dtype, valid_field_names = _point_fields_to_dtypes(msg.fields, msg.point_step)
    array = np.frombuffer(msg.data, dtype)
    if len(array.dtype.names) != len(valid_field_names):
        array = array[valid_field_names]

    array = array.reshape((msg.height, msg.width))
    array_dict = {name: array[name] for name in array.dtype.fields.keys()}

    result = {"xyz": np.stack([array_dict.pop(k) for k in ["x", "y", "z"]], axis=-1)}

    if "rgb" in array_dict:
        rgb_uint32 = array_dict.pop("rgb")
        if not np.issubdtype(rgb_uint32.dtype, np.uint32):
            rgb_uint32.dtype = np.uint32

        shift = np.array([0, 8, 16], np.uint8)
        result["rgb"] = np.asarray((rgb_uint32[..., None] >> shift) & 255, dtype=np.uint8)

    if "rgba" in array_dict:
        rgba_uint32 = array_dict.pop("rgba")
        if not np.issubdtype(rgba_uint32.dtype, np.uint32):
            rgba_uint32.dtype = np.uint32

        shift = np.array([0, 8, 16, 24], np.uint8)
        result["rgba"] = np.asarray((rgba_uint32[..., None] >> shift) & 255, dtype=np.uint8)

    result.update(array_dict)

    return result


# ======================================================================================================================
#
#   Private Methods
#
# ======================================================================================================================
def _point_fields_to_dtypes(fields: list[sensor_msgs.PointField], point_step: int) -> tuple[np.dtype, list[str]]:
    """
    Converts list of sensor_msgs.PointField to np.dtype.
    Additionally, returns valid field names because PointCloud2 sometimes has dummy fields and data.

    Args:
        fields:
        point_step:

    Returns:
        numpy dtype.
        field name of non-dummy.
    """
    offset = 0
    dtype_list = []
    valid_field_names = []
    for field in fields:
        if offset != field.offset:
            for k in range(field.offset - offset):
                dtype_list.append((f"__EMPTY__{offset + k}", np.uint8))
            offset = field.offset

        dtype = PointType.point_fields_to_np_dtypes(field.datatype)
        if field.count != 1:
            dtype = np.dtype((dtype, field.count))

        dtype_list.append((field.name, dtype))
        valid_field_names.append(field.name)
        offset += PointType.point_field_to_size(field.datatype) * field.count

    if offset != point_step:
        for k in range(point_step - offset):
            dtype_list.append((f"__EMPTY__{offset + k}", np.uint8))

    return np.dtype(dtype_list), valid_field_names
