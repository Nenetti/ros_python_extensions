from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Final, Type, TypeVar, final

import rospy

from . import Timeout
from .topic_server import TopicServer


class AbstractTopic(ABC):

    _name: Final[str]
    _data_class: Final[Type[rospy.Message]]

    def __init__(self, name: str, data_class: Type[rospy.Message]):
        TopicServer.register_topic(name, data_class)
        self._name = name
        self._data_class = data_class

    # ==================================================================================================================
    #
    #   Public Methods
    #
    # ==================================================================================================================
    @final
    def wait_for_connections(self, timeout: (float | rospy.Duration)) -> None:
        timer = Timeout(timeout)
        r = rospy.Rate(100)
        while not self.has_connections():
            if rospy.is_shutdown():
                raise rospy.exceptions.ROSInterruptException("rospy shutdown")
            if timer.is_timeout():
                raise rospy.exceptions.ROSException("timeout")
            r.sleep()

    @final
    def has_connections(self) -> bool:
        return 0 < self.get_num_connections()

    @abstractmethod
    def is_active(self) -> bool:
        pass

    @abstractmethod
    def get_num_connections(self) -> int:
        pass

    @abstractmethod
    def unregister(self) -> None:
        pass

    @abstractmethod
    def reinitialization(self) -> AbstractTopic:
        pass
