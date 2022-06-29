from __future__ import annotations

from typing import Any, Final, Type

import genpy
import message_filters

__all__ = [
    "ApproximateTimeSynchronizerFactory",
]


class ApproximateTimeSynchronizerFactory:

    _subscriber_kwargs: Final[dict[str, dict[Any]]]
    _synchronizer: (message_filters.ApproximateTimeSynchronizer | None)
    _queue_size: Final[int]
    _synchronize_time_error: Final[float]

    def __init__(self, synchronize_time_error=0.1, queue_size=10) -> None:
        self._subscriber_kwargs = {}
        self._synchronizer = None
        self._synchronize_time_error = synchronize_time_error
        self._queue_size = queue_size

    # ==================================================================================================
    #
    #   Public Methods
    #
    # ==================================================================================================
    def register_subscriber(self, topic: str, data_class: Type[genpy.Message], queue_size: int = 1) -> None:
        self._subscriber_kwargs[topic] = {"name": topic, "data_class": data_class, "queue_size": queue_size}

    def create(self):
        filtered_subscribers = [message_filters.Subscriber(**kwargs) for kwargs in self._subscriber_kwargs.values()]
        self._synchronizer = ApproximateTimeSynchronizer(
            filtered_subscribers, self._queue_size, self._synchronize_time_error
        )


class ApproximateTimeSynchronizer(message_filters.ApproximateTimeSynchronizer):

    _subscribers: Final[list[message_filters.Subscriber]]

    def __init__(self, *args, **kwargs):
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
