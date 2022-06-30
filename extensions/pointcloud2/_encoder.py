from __future__ import annotations

import numpy as np
import rospy
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs

from ._point_type import PointType

__all__ = [
    "numpy_to_pointcloud2",
]


# ======================================================================================================================
#
#   Private Methods
#
# ======================================================================================================================
def numpy_to_pointcloud2(
        xyz: np.ndarray,
        rgb: np.ndarray = None,
        rgba: np.ndarray = None,
        **kwargs: np.ndarray
) -> sensor_msgs.PointCloud2:
    """
    Converts xyz array to sensor_msgs.PointCloud2.

    Args:
        xyz: [H, W, 3] shape.
        rgb: [H, W, 3] shape. uint8.
        rgba: [H, W, 4] shape. uint8.
        kwargs: [H, W] shape.

    Returns:

    """
    if (rgb is not None) and (rgba is not None):
        raise ValueError()

    dtypes = {"x": np.float32, "y": np.float32, "z": np.float32}
    if rgb is not None:
        dtypes["rgb"] = np.uint32
    if rgba is not None:
        dtypes["rgba"] = np.uint32
    for key, value in kwargs.items():
        if key in dtypes:
            raise ValueError()
        dtypes[key] = value.dtype.type

    array_dtype = np.dtype([(k, v) for k, v in dtypes.items()])
    array = np.empty(xyz.shape[:-1], array_dtype)

    xyz = xyz.astype(np.float32)
    array["x"] = xyz[..., 0]
    array["y"] = xyz[..., 1]
    array["z"] = xyz[..., 2]

    if rgb is not None:
        rgb = rgb.astype(np.uint32)
        array["rgb"] = ((rgb[..., 0] << 0) | (rgb[..., 1] << 8) | (rgb[..., 2] << 16))
    if rgba is not None:
        rgba = rgba.astype(np.uint32)
        array["rgba"] = ((rgba[..., 0] << 0) | (rgba[..., 1] << 8) | (rgba[..., 2] << 16) | (rgba[..., 3] << 24))
    for key, value in kwargs.items():
        array[key] = value

    return _numpy_to_ros_msgs(array)


# ======================================================================================================================
#
#   Private Methods
#
# ======================================================================================================================
def _numpy_to_ros_msgs(points: np.ndarray, force_dense: bool = False) -> sensor_msgs.PointCloud2:
    """
    Converts numpy array to sensor_msgs.PointCloud2.

    Args:
        points: [H, W] or [N] shape.
        force_dense:
    """
    if force_dense:
        y = np.all(np.stack([np.isfinite(points[name]) for name in points.dtype.names], axis=0), axis=0)
        is_dense = np.all(y).item()
        if not is_dense:
            points = points[y]
    else:
        is_dense = False

    if points.ndim != 2:
        points = points.reshape(1, -1)

    header = std_msgs.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = ""

    fields, point_step = _dtype_to_fields(points.dtype)
    height, width = points.shape

    cloud_msg = sensor_msgs.PointCloud2()
    cloud_msg.header = header
    cloud_msg.height = height
    cloud_msg.width = width
    cloud_msg.fields = fields
    cloud_msg.is_bigendian = False
    cloud_msg.point_step = point_step
    cloud_msg.row_step = point_step * width
    cloud_msg.data = points.tostring()
    cloud_msg.is_dense = is_dense

    return cloud_msg


def _dtype_to_fields(field_dtype: np.dtype) -> tuple[list[sensor_msgs.PointField], int]:
    """
    Converts np.dtype to list of sensor_msgs.PointField.
    Additionally, returns the number of bytes in 1 points.

    Args:
        field_dtype:
    """
    fields = []
    for name, (np_dtype, offset) in field_dtype.fields.items():
        pf = sensor_msgs.PointField()
        pf.name = name
        pf.offset = offset
        pf.datatype = PointType.np_dtypes_to_point_fields(np_dtype)
        pf.count = 1
        fields.append(pf)
    return fields, field_dtype.itemsize
