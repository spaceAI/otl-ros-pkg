#!/usr/bin/env python
import roslib
roslib.load_manifest('otl_roomba')

import rospy
from otl_roomba.roombaif import *

class RoombaBaseNode:
    """
    roomba interface for ROS (virtual class)
    """
    def __init__(self, rmb):
        self._rmb = rmb
        self._rmb.setup()

    def _send_clean_cb(self, command):
        if command.data:
            self._rmb.clean_on()
        else:
            self._rmb.clean_off()

    def _send_dock_cb(self, command):
        if command.data:
            self._rmb.dock()

    def close(self):
        self._rmb.close()


if __name__ == '__main__':
    rospy.init_node('roomba')
    rmb = RoombaIf()
    rn = RoombaBaseNode(rmb)
