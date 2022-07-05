from __future__ import annotations

from typing import Type

import rospy
import rostopic


class TopicServer:

    _topic_dict: dict[str, Type[rospy.Message]] = {}

    @classmethod
    def register_topic(cls, topic: str, data_class: Type[rospy.Message]):
        resolved_topic = rospy.names.resolve_name(topic)
        cls._assert_type_check(resolved_topic, data_class)
        if resolved_topic not in cls._topic_dict:
            cls._topic_dict[resolved_topic] = data_class

    @classmethod
    def assert_type_check(cls, topic: str, data_class: Type[rospy.Message]):
        resolved_topic = rospy.names.resolve_name(topic)
        cls._assert_type_check(resolved_topic, data_class)

    @classmethod
    def _assert_type_check(cls, resolved_topic: str, data_class: Type[rospy.Message]):
        if (resolved_topic in cls._topic_dict) and (data_class != cls._topic_dict[resolved_topic]):
            raise rospy.ROSException("msg type miss")

        msg_class, real_topic, msg_eval = rostopic.get_topic_class(resolved_topic)
        if (msg_class is not None) and (msg_class != data_class):
            raise rospy.ROSException("msg type miss")
