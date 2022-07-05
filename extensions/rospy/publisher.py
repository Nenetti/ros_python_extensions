from __future__ import annotations

from typing import Final, Generic, TypeVar

import rospy

__all__ = [
    "Publisher"
]

from ._abstract_topic import AbstractTopic

T = TypeVar("T")


class Publisher(AbstractTopic, Generic[T]):
    _subscriber_listener: Final[rospy.SubscribeListener]
    _tcp_nodelay: Final[bool]
    _latch: Final[bool]
    _headers: Final[dict]
    _queue_size: Final[int]

    _msg: None

    def __init__(
            self, name, data_class, subscriber_listener=None, tcp_nodelay=False,
            latch=False, headers=None, queue_size=1
    ):
        super(Publisher, self).__init__(name, data_class)
        self._publisher = rospy.Publisher(
            name, data_class, subscriber_listener, tcp_nodelay, latch, headers, queue_size
        )
        self._subscriber_listener = subscriber_listener
        self._tcp_nodelay = tcp_nodelay
        self._latch = latch
        self._headers = headers
        self._queue_size = queue_size

    def publish(self, msg: T) -> None:
        self._publisher.publish(msg)

    def is_active(self) -> bool:
        return self._publisher.impl is not None

    def get_num_connections(self) -> int:
        return self._publisher.get_num_connections()

    def unregister(self) -> None:
        self._publisher.unregister()

    def reinitialization(self) -> Publisher[T]:
        if self.is_active():
            self.unregister()
        return self.__class__(
            self._name, self._data_class, self._subscriber_listener, self._tcp_nodelay,
            self._latch, self._headers, self._queue_size
        )
