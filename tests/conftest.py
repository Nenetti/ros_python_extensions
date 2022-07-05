import pytest
import rospy.exceptions

from extensions.rospy import Roscore


@pytest.fixture(scope="session", autouse=True)
def fixture_session():
    roscore_process = Roscore(stdout=False, autostart=True)
    rospy.init_node("pytest_node", anonymous=True)
    yield
    rospy.signal_shutdown("finished pytest")
    roscore_process.terminate()
