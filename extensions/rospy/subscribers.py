from __future__ import annotations

import threading
from typing import Callable, Final, Generic, Type, TypeVar

import rospy
import rospy.impl.tcpros_base
from rospy.impl.tcpros_base import DEFAULT_BUFF_SIZE

from extensions.rospy import Timeout
from ._abstract_topic import AbstractTopic

__all__ = [
    "Subscriber",
    "QueueSubscriber"
]
T = TypeVar('T')


class Subscriber(AbstractTopic, Generic[T]):

    _callback: Final[Callable[[rospy.Message, ...], None]]
    _callback_args: Final[(tuple[...] | None)]
    _queue_size: Final[int]
    _buff_size: Final[int]
    _tcp_nodelay: Final[bool]
    _subscriber: Final[rospy.Subscriber]

    _msg: None

    def __init__(
            self, name: str, data_class: Type[rospy.Message], callback: Callable[[rospy.Message, ...], None],
            callback_args: (tuple[...] | None) = None, queue_size: int = 1,
            buff_size: int = DEFAULT_BUFF_SIZE, tcp_nodelay: bool = False
    ):
        super(Subscriber, self).__init__(name, data_class)
        self._callback = callback
        self._callback_args = callback_args
        self._queue_size = queue_size
        self._buff_size = buff_size
        self._tcp_nodelay = tcp_nodelay
        self._msg = None
        self._subscriber = rospy.Subscriber(
            name, data_class, callback, callback_args, queue_size, buff_size, tcp_nodelay
        )

    # ==================================================================================================================
    #
    #   Public Methods
    #
    # ==================================================================================================================
    def is_active(self) -> bool:
        return self._subscriber.impl is not None

    def get_num_connections(self) -> int:
        return self._subscriber.get_num_connections()

    def unregister(self) -> None:
        self._subscriber.unregister()

    def reinitialization(self) -> Subscriber[T]:
        if self.is_active():
            self.unregister()
        return self.__class__(
            self._name, self._data_class, self._callback, self._callback_args,
            self._queue_size, self._buff_size, self._tcp_nodelay
        )


class QueueSubscriber(Subscriber):

    _lock: Final[threading.Lock]
    _has_header: Final[bool]
    _msg: (tuple[rospy.Time, rospy.Message] | rospy.Message | None)

    def __init__(
            self, name: str, data_class: Type[rospy.Message], queue_size: int = 1,
            buff_size: int = DEFAULT_BUFF_SIZE, tcp_nodelay: bool = False
    ):
        super(QueueSubscriber, self).__init__(
            name, data_class, self._callback_wrapper, None, queue_size, buff_size, tcp_nodelay
        )
        self._lock = threading.Lock()
        self._has_header = hasattr(data_class, "header")
        self._msg = None

    def wait_for_message(self, stamp: rospy.Time = None, timeout: (float | rospy.Duration | None) = None) -> T:
        if self.has_message(stamp):
            return self.pop_message()

        r = rospy.Rate(100)
        if timeout is None:
            while not self.has_message(stamp):
                if rospy.is_shutdown():
                    raise rospy.exceptions.ROSInterruptException("rospy shutdown")
                r.sleep()

        else:
            timer = Timeout(timeout)
            while not self.has_message(stamp):
                if rospy.is_shutdown():
                    raise rospy.exceptions.ROSInterruptException("rospy shutdown")
                if timer.is_timeout():
                    raise rospy.exceptions.ROSException("timeout")
                r.sleep()

        return self.pop_message()

    def pop_message(self) -> T:
        with self._lock:
            msg = self._msg
            self._msg = None

        if self._has_header:
            return msg
        else:
            return msg[1]

    def has_message(self, stamp: (rospy.Time | None) = None) -> bool:
        with self._lock:
            if stamp is None:
                return self._msg is not None

            if self._msg is None:
                return False

            if self._has_header:
                return getattr(self._msg, "header") >= stamp
            else:
                return self._msg[0] >= stamp

    def reinitialization(self) -> QueueSubscriber[T]:
        if self.is_active():
            self.unregister()
        return self.__class__(self._name, self._data_class, self._queue_size, self._buff_size, self._tcp_nodelay)

    # ==================================================================================================================
    #
    #   ROS Callback
    #
    # ==================================================================================================================
    def _callback_wrapper(self, msg: rospy.Message) -> None:
        with self._lock:
            self._msg = msg if self._has_header else (rospy.Time.now(), msg)
