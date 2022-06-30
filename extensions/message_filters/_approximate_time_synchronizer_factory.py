from __future__ import annotations

from typing import Any, Callable, Final, Type

import genpy
import message_filters

__all__ = [
    "ApproximateTimeSynchronizerFactory",
]


class ApproximateTimeSynchronizerFactory:

    _subscriber_kwargs: Final[dict[str, dict[Any]]]
    _queue_size: Final[int]
    _synchronize_time_error: Final[float]
    _callback: Final[Callable]

    def __init__(self, callback, synchronize_time_error=0.1, queue_size=10) -> None:
        self._subscriber_kwargs = {}
        self._synchronize_time_error = synchronize_time_error
        self._queue_size = queue_size
        self._callback = callback

    # ==================================================================================================
    #
    #   Public Methods
    #
    # ==================================================================================================
    def register_subscriber(
            self, topic: str, data_class: Type[genpy.Message], queue_size: int = 1
    ) -> ApproximateTimeSynchronizerFactory:
        self._subscriber_kwargs[topic] = {"data_class": data_class, "queue_size": queue_size}
        return self

    def create(self) -> ApproximateTimeSynchronizer:
        filtered_subscribers = [
            message_filters.Subscriber(topic, **kwargs) for topic, kwargs in self._subscriber_kwargs.items()
        ]
        synchronizer = ApproximateTimeSynchronizer(filtered_subscribers, self._queue_size, self._synchronize_time_error)
        synchronizer.registerCallback(self._callback)
        return synchronizer


class ApproximateTimeSynchronizer(message_filters.ApproximateTimeSynchronizer):

    _subscribers: Final[list[message_filters.Subscriber]]

    def __init__(self, *args, **kwargs) -> None:
        super(ApproximateTimeSynchronizer, self).__init__(*args, **kwargs)
        self._subscribers = args[0]

    # ==================================================================================================
    #
    #   Public Methods
    #
    # ==================================================================================================
    def unregister(self) -> None:
        for subscriber in self._subscribers:
            subscriber.sub.unregister()
