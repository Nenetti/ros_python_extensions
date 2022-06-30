from __future__ import annotations

import threading
from typing import Any, Callable, Final, Type

import genpy
import rospy

__all__ = [
    "SubscriberFactory",
]


class SubscriberFactory:

    _topic: Final[str]
    _data_class: Final[Type[genpy.Message]]
    _queue_size: Final[int]
    _callback_latest: Final[bool]

    def __init__(
            self, topic: str, data_class: Type[genpy.Message], callback: Callable, queue_size: int = 1,
            callback_latest: bool = False
    ) -> None:
        self._topic = topic
        self._data_class = data_class
        self._callback = callback
        self._queue_size = queue_size
        self._callback_latest = callback_latest

    # ==================================================================================================================
    #
    #   Public Methods
    #
    # ==================================================================================================================
    def create(self) -> rospy.Subscriber:
        if self._callback_latest:
            return ThreadSubscriber(self._callback, self._topic, self._data_class, queue_size=self._queue_size)

        return rospy.Subscriber(self._topic, self._data_class, self._callback, queue_size=self._queue_size)


class ThreadSubscriber(rospy.Subscriber):

    _lock: Final[threading.Lock]
    _callback: Final[Callable]

    _msg_queue: (genpy.Message | None)
    _thread: (threading.Thread | None)

    def __init__(self, callback: Callable, *args, **kwargs) -> None:
        super(ThreadSubscriber, self).__init__(*args, **kwargs)
        self._lock = threading.Lock()
        self._callback = callback
        self._msg_queue = None
        self._unregister_request = False
        self._thread = threading.Thread(target=self._thread_loop).start()

    # ==================================================================================================================
    #
    #   Override Methods
    #
    # ==================================================================================================================
    def unregister(self) -> None:
        super(ThreadSubscriber, self).unregister()
        self._unregister_request = True
        self._thread.join()

    # ==================================================================================================================
    #
    #   ROS Callback
    #
    # ==================================================================================================================
    def _callback_wrapper(self, msg: Any) -> None:
        with self._lock:
            self._msg_queue = msg

    # ==================================================================================================================
    #
    #   Other Thread Methods
    #
    # ==================================================================================================================
    def _thread_loop(self) -> None:
        r = rospy.Rate(100)
        while (not rospy.is_shutdown()) and (not self._unregister_request):
            r.sleep()

            with self._lock:
                if self._msg_queue is None:
                    continue

                msg = self._msg_queue
                self._msg_queue = None

            self._callback(msg)
