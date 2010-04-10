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

    def _sendVelCB(self, command):
        self._rmb.setVel(command.vel, command.rad)

    def _sendCleanCB(self, command):
        if command.data:
            self._rmb.cleanOn()
        else:
            self._rmb.cleanOff()

    def close(self):
        self._rmb.close()

    def main(self):
        rospy.init_node('roomba')
        rospy.Subscriber('roomba/command', VelRad, self._sendVelCB, queue_size=1)
        rospy.Subscriber('roomba/clean', Bool, self._sendCleanCB, queue_size=1)
        rospy.spin()
        self.close()

if __name__ == '__main__':
    rn = RoombaNode()
    rn.main()

