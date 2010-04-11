#!/usr/bin/env python
import roslib
roslib.load_manifest('otl_roomba')

import rospy
from otl_roomba.roombaif import *
from otl_roomba.msg import VelRad
from std_msgs.msg import Bool


class RoombaNode:
    """
    roomba interface for ROS
    """
    def __init__(self):
        self._rmb = RoombaIf()
        self._rmb.setup()

    def _send_vel_cb(self, command):
        self._rmb.set_vel(command.vel, command.rad)

    def _send_clean_cb(self, command):
        if command.data:
            self._rmb.clean_on()
        else:
            self._rmb.clean_off()

    def close(self):
        self._rmb.close()

    def main(self):
        rospy.init_node('roomba')
        rospy.Subscriber('roomba/command',
                         VelRad, self._send_vel_cb, queue_size=1)
        rospy.Subscriber('roomba/clean',
                         Bool, self._send_clean_cb, queue_size=1)
        rospy.spin()
        self.close()


if __name__ == '__main__':
    rn = RoombaNode()
    rn.main()
