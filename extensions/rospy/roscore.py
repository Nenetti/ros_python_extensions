import subprocess
import time

import rosgraph
import rospy

__all__ = [
    "Roscore",
]


class Roscore:

    def __init__(self, stdout: bool = True, autostart: bool = False):
        self._process = None
        self._stdout = stdout
        if autostart:
            self.run()

    def run(self) -> None:
        if self.is_running():
            raise rospy.exceptions.ROSInitException("roscore is already running.")
        if self._stdout:
            self._process = subprocess.Popen(["roscore"])
        else:
            self._process = subprocess.Popen(["roscore"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        while not self.is_running():
            time.sleep(0.01)

    def terminate(self) -> None:
        self._process.terminate()
        self._process.wait()

    @classmethod
    def is_running(cls) -> bool:
        return rosgraph.masterapi.is_online()
