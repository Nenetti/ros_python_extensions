import time

import pytest
import rospy
import std_msgs.msg as std_msgs

import extensions
from extensions.rospy import Timeout


@pytest.fixture
def topic():
    return f"/pytest/{int(time.time() * (10 ** 6))}"


def _dummy_callback(msg):
    pass


class TestPublisherSubscriber:

    @staticmethod
    def test_difference_dataclass_exception(topic):
        extensions.rospy.Publisher(topic, std_msgs.Int32)
        with pytest.raises(Exception):
            extensions.rospy.Subscriber(topic, std_msgs.String, _dummy_callback)

    @staticmethod
    def test_pub_sub_connections(topic):
        publisher = extensions.rospy.Publisher(topic, std_msgs.Int32)
        assert not publisher.has_connections()

        subscriber = extensions.rospy.Subscriber(topic, std_msgs.Int32, _dummy_callback)
        subscriber.wait_for_connections(timeout=1.0)

        assert publisher.has_connections()

    @staticmethod
    def test_sub_pub_connections(topic):
        subscriber = extensions.rospy.Subscriber(topic, std_msgs.Int32, _dummy_callback)
        assert not subscriber.has_connections()

        publisher = extensions.rospy.Publisher(topic, std_msgs.Int32)
        publisher.wait_for_connections(timeout=1.0)

        assert subscriber.has_connections()

    @staticmethod
    def test_subscriber_callback(topic):
        subscribe_msg = None

        def _callback(msg):
            nonlocal subscribe_msg
            subscribe_msg = msg

        publisher = extensions.rospy.Publisher(topic, std_msgs.Int32)
        subscriber = extensions.rospy.Subscriber(topic, std_msgs.Int32, _callback)
        subscriber.wait_for_connections(timeout=1.0)

        publish_msg = std_msgs.Int32(1)
        publisher.publish(publish_msg)
        timeout = Timeout(1.0)
        r = rospy.Rate(100)
        while (not rospy.is_shutdown()) and (subscribe_msg is None):
            assert not timeout.is_timeout()
            r.sleep()

        assert type(publish_msg) is type(subscribe_msg)
        assert publish_msg.data == subscribe_msg.data


class TestQueueSubscriber:

    @staticmethod
    def test_subscriber_callback(topic):
        publisher = extensions.rospy.Publisher(topic, std_msgs.Int32)
        subscriber = extensions.rospy.QueueSubscriber(topic, std_msgs.Int32)
        subscriber.wait_for_connections(timeout=1.0)

        publish_msg = std_msgs.Int32(1)
        publisher.publish(publish_msg)
        subscribe_msg = subscriber.wait_for_message(timeout=1.0)

        assert type(publish_msg) is type(subscribe_msg)
        assert publish_msg.data == subscribe_msg.data
