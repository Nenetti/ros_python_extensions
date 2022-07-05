from __future__ import annotations

import time

import pytest
import rospy

import extensions.message_filters
import extensions.rospy
from extensions.message_filters import ApproximateTimeSynchronizerBuilder
import sensor_msgs.msg as sensor_msgs


@pytest.fixture
def topic1():
    return f"/pytest/{int(time.time() * (10 ** 6))}"


@pytest.fixture
def topic2():
    return f"/pytest/{int(time.time() * (10 ** 6))}"


class TestApproximateTimeSynchronizerBuilder:

    @staticmethod
    def test_approximate_time_synchronizer_callback(topic1, topic2):
        subscribe_msg1: (sensor_msgs.Image | None) = None
        subscribe_msg2: (sensor_msgs.CompressedImage | None) = None

        def _callback(msg1, msg2):
            nonlocal subscribe_msg1, subscribe_msg2
            subscribe_msg1 = msg1
            subscribe_msg2 = msg2

        synchronizer = (ApproximateTimeSynchronizerBuilder(_callback, synchronize_error_seconds=0.1)
                        .add_topic(topic1, sensor_msgs.Image, queue_size=1)
                        .add_topic(topic2, sensor_msgs.CompressedImage, queue_size=1)
                        .build())
        publisher1 = extensions.rospy.Publisher(topic1, sensor_msgs.Image, queue_size=1)
        publisher2 = extensions.rospy.Publisher(topic2, sensor_msgs.CompressedImage, queue_size=1)
        publisher1.wait_for_connections(timeout=1.0)
        publisher2.wait_for_connections(timeout=1.0)

        publish_msg1 = sensor_msgs.Image()
        publish_msg2 = sensor_msgs.CompressedImage()
        publish_msg1.header.stamp = rospy.Time.now()
        publish_msg2.header.stamp = rospy.Time.now()
        publisher1.publish(publish_msg1)
        publisher2.publish(publish_msg2)

        r = rospy.Rate(100)
        timeout = extensions.rospy.Timeout(1.0)
        while (not rospy.is_shutdown()) and (subscribe_msg1 is None):
            assert not timeout.is_timeout()
            r.sleep()

        assert type(publish_msg1) is type(subscribe_msg1)
        assert type(publish_msg2) is type(subscribe_msg2)
        assert publish_msg2.data == subscribe_msg2.data
        assert publish_msg2.data == subscribe_msg2.data
