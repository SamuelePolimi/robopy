#!/usr/bin/env python
"""
Here there is the interface with ROS.

"""
import rospy


def activate_listener():
    rospy.init_node("listener", anonymous=True)


class RosListener:

    def __init__(self):
        self.ready = False

    def _internal_callback(self, data):
        self._callback(data)
        self.ready = True

    def _callback(self, data):
        raise NotImplementedError()

    def get_callback(self):
        return lambda x: self._internal_callback(x)
