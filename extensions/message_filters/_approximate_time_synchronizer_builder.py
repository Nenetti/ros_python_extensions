from __future__ import annotations

from typing import Any, Callable, Final, Type

import message_filters
import rospy

from rospy.impl.tcpros_base import DEFAULT_BUFF_SIZE
from ..rospy.topic_server import TopicServer

__all__ = [
    "ApproximateTimeSynchronizerBuilder",
]


class ApproximateTimeSynchronizerBuilder:

    _subscriber_kwargs: Final[list[dict[str, Any]]]
    _callback: Final[Callable]
    _callback_args: Final[(tuple[...] | None)]
    _synchronize_error_seconds: Final[float]
    _queue_size: Final[int]
    _allow_headerless: Final[bool]
    _reset: Final[bool]

    def __init__(
            self, callback: Callable[[...], None], synchronize_error_seconds: float,
            callback_args=(), queue_size: int = 10,
            allow_headerless: bool = False, reset: bool = False
    ) -> None:
        self._subscriber_kwargs = []
        self._callback = callback
        self._callback_args = callback_args
        self._synchronize_error_seconds = synchronize_error_seconds
        self._queue_size = queue_size
        self._allow_headerless = allow_headerless
        self._reset = reset

    def add_topic(
            self, name: str, data_class: Type[rospy.Message], callback_args: (tuple[...] | None) = None,
            queue_size: int = 1, buff_size: int = DEFAULT_BUFF_SIZE, tcp_nodelay: bool = False
    ) -> ApproximateTimeSynchronizerBuilder:
        kwargs = dict(
            name=name, data_class=data_class, callback_args=callback_args, queue_size=queue_size,
            buff_size=buff_size, tcp_nodelay=tcp_nodelay
        )
        self._subscriber_kwargs.append(kwargs)
        return self

    def build(self) -> ApproximateTimeSynchronizer:
        synchronizer = ApproximateTimeSynchronizer(
            self._subscriber_kwargs, self._callback, self._synchronize_error_seconds, self._callback_args,
            self._queue_size, self._allow_headerless, self._reset
        )
        return synchronizer


class ApproximateTimeSynchronizer:

    _subscriber_kwargs: Final[list[dict[str, Any]]]
    _callback: Final[Callable]
    _callback_args: Final[(tuple[...] | None)]
    _synchronize_error_seconds: Final[float]
    _queue_size: Final[int]
    _allow_headerless: Final[bool]
    _reset: Final[bool]

    _subscribers: Final[list[_Subscriber]]
    _synchronizer: Final[message_filters.ApproximateTimeSynchronizer]

    def __init__(
            self, subscriber_kwargs: list[dict[str, Any]], callback: Callable[[...], None],
            synchronize_error_seconds: float, callback_args: (tuple[...] | None) = (), queue_size: int = 10,
            allow_headerless: bool = False, reset: bool = False
    ) -> None:
        self._subscriber_kwargs = subscriber_kwargs
        self._callback = callback
        self._callback_args = callback_args
        self._synchronize_error_seconds = synchronize_error_seconds
        self._queue_size = queue_size
        self._allow_headerless = allow_headerless
        self._reset = reset
        self._subscribers = [_Subscriber(**kwargs) for kwargs in subscriber_kwargs]
        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            self._subscribers, queue_size, synchronize_error_seconds, allow_headerless, reset
        )
        self._synchronizer.registerCallback(callback, *self._callback_args)

    def is_active(self) -> bool:
        return all([subscriber.sub.impl is not None for subscriber in self._subscribers])

    def unregister(self) -> None:
        for subscriber in self._subscribers:
            subscriber.sub.unregister()

    def reinitialization(self) -> ApproximateTimeSynchronizer:
        if any([subscriber.sub.impl is not None for subscriber in self._subscribers]):
            self.unregister()

        return self.__class__(
            self._subscriber_kwargs, self._callback, self._synchronize_error_seconds, self._callback_args,
            self._queue_size, self._allow_headerless, self._reset
        )


class _Subscriber(message_filters.Subscriber):

    def __init__(self, name: str, data_class: Type[rospy.Message], **kwargs):
        TopicServer.register_topic(name, data_class)
        super(_Subscriber, self).__init__(name, data_class, **kwargs)
