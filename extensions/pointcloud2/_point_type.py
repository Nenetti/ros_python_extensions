from __future__ import annotations

from typing import Final, Type

import numpy as np
from sensor_msgs.msg import PointField


class PointType:
    """
    http://wiki.ros.org/pcl/Overview#Common_PointCloud2_field_names
    """

    _point_fields_to_np_dtypes: Final = {
        PointField.INT8: np.int8,
        PointField.UINT8: np.uint8,
        PointField.INT16: np.int16,
        PointField.UINT16: np.uint16,
        PointField.INT32: np.int32,
        PointField.UINT32: np.uint32,
        PointField.FLOAT32: np.float32,
        PointField.FLOAT64: np.float64
    }

    _np_dtypes_to_point_fields: Final = {v: k for k, v in _point_fields_to_np_dtypes.items()}

    _point_field_to_size: Final = {
        PointField.INT8: 1,
        PointField.UINT8: 1,
        PointField.INT16: 2,
        PointField.UINT16: 2,
        PointField.INT32: 4,
        PointField.UINT32: 4,
        PointField.FLOAT32: 4,
        PointField.FLOAT64: 8
    }

    @classmethod
    def point_fields_to_np_dtypes(cls, field: int) -> Type[np.dtype]:
        return cls._point_fields_to_np_dtypes[field]

    @classmethod
    def np_dtypes_to_point_fields(cls, dtype: np.dtype) -> int:
        return cls._np_dtypes_to_point_fields[dtype.type]

    @classmethod
    def point_field_to_size(cls, field: int) -> int:
        return cls._point_field_to_size[field]
