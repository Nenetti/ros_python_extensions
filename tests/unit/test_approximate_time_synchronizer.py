from __future__ import annotations

import time

import pytest
import sensor_msgs.msg as sensor_msgs

from extensions.message_filters import ApproximateTimeSynchronizerBuilder


@pytest.fixture
def topic1():
    return f"/pytest/{int(time.time() * (10 ** 6))}"


@pytest.fixture
def topic2():
    return f"/pytest/{int(time.time() * (10 ** 6))}"


def _dummy_callback(msg1, msg2):
    pass


class TestApproximateTimeSynchronizerBuilder:

    @staticmethod
    def test_difference_dataclass_exception(topic1, topic2):
        (ApproximateTimeSynchronizerBuilder(_dummy_callback, synchronize_error_seconds=0.1)
         .add_topic(topic1, sensor_msgs.Image, queue_size=1)
         .add_topic(topic2, sensor_msgs.CompressedImage, queue_size=1)
         .build())
        with pytest.raises(Exception):
            (ApproximateTimeSynchronizerBuilder(_dummy_callback, synchronize_error_seconds=0.1)
             .add_topic(topic1, sensor_msgs.CompressedImage, queue_size=1)
             .add_topic(topic2, sensor_msgs.Image, queue_size=1)
             .build())

    @staticmethod
    def test_active_status(topic1, topic2):
        synchronizer = (ApproximateTimeSynchronizerBuilder(_dummy_callback, synchronize_error_seconds=0.1)
                        .add_topic(topic1, sensor_msgs.Image, queue_size=1)
                        .add_topic(topic2, sensor_msgs.CompressedImage, queue_size=1)
                        .build())

        assert synchronizer.is_active()
        synchronizer.unregister()
        assert not synchronizer.is_active()

    @staticmethod
    def test_reinitialization(topic1, topic2):
        synchronizer1 = (ApproximateTimeSynchronizerBuilder(_dummy_callback, synchronize_error_seconds=0.1)
                         .add_topic(topic1, sensor_msgs.Image, queue_size=1)
                         .add_topic(topic2, sensor_msgs.CompressedImage, queue_size=1)
                         .build())
        synchronizer2 = synchronizer1.reinitialization()
        assert not synchronizer1.is_active()
        assert synchronizer2.is_active()
