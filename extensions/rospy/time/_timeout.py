from __future__ import annotations

from typing import Final

import rospy


class Timeout:

    _time: Final[rospy.Duration]
    _limit_time: rospy.Time

    def __init__(self, second: (float | rospy.Duration)):
        if isinstance(second, rospy.Duration):
            self._time = second
        else:
            secs = int(second)
            nsecs = int((second - secs) * 1000000000)
            self._time = rospy.Duration(secs, nsecs)
        self.reset()

    def reset(self) -> None:
        self._limit_time = rospy.get_rostime() + self._time

    def left_time(self) -> float:
        return (self._limit_time - rospy.get_rostime()).to_sec()

    def is_timeout(self) -> bool:
        if (self._limit_time - rospy.get_rostime()) <= rospy.Duration(0):
            return True

        return False
