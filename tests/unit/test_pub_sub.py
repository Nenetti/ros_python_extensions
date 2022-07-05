import time

import pytest
import std_msgs.msg as std_msgs

import extensions.rospy


@pytest.fixture
def topic():
    return f"/pytest/{int(time.time() * (10 ** 6))}"


def _dummy_callback(msg):
    pass


class TestSubscriber:

    @staticmethod
    def test_difference_dataclass_exception(topic):
        extensions.rospy.Subscriber(topic, std_msgs.Int32, _dummy_callback)
        with pytest.raises(Exception):
            extensions.rospy.Subscriber(topic, std_msgs.String, _dummy_callback)

    @staticmethod
    def test_active_status(topic):
        subscriber = extensions.rospy.Subscriber(topic, std_msgs.Int32, _dummy_callback)
        assert subscriber.is_active()
        subscriber.unregister()
        assert not subscriber.is_active()

    @staticmethod
    def test_reinitialization(topic):
        subscriber1 = extensions.rospy.Subscriber(topic, std_msgs.Int32, _dummy_callback)
        subscriber2 = subscriber1.reinitialization()
        assert not subscriber1.is_active()
        assert subscriber2.is_active()


class TestPublisher:

    @staticmethod
    def test_difference_dataclass_exception(topic):
        extensions.rospy.Publisher(topic, std_msgs.Int32)
        with pytest.raises(Exception):
            extensions.rospy.Publisher(topic, std_msgs.String)

    @staticmethod
    def test_active_status(topic):
        publisher = extensions.rospy.Publisher(topic, std_msgs.Int32)
        assert publisher.is_active()
        publisher.unregister()
        assert not publisher.is_active()

    @staticmethod
    def test_reinitialization(topic):
        publisher1 = extensions.rospy.Publisher(topic, std_msgs.Int32)
        publisher2 = publisher1.reinitialization()
        assert not publisher1.is_active()
        assert publisher2.is_active()
