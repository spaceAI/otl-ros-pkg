#!/usr/bin/env python
import roslib
roslib.load_manifest('otl_roomba')

import rospy
from otl_roomba.roombaif import *
from roomba_base_node import *
from otl_roomba.msg import VelRad
from std_msgs.msg import Bool


class RoombaNode(RoombaBaseNode):
    """
    roomba interface for ROS (controlled by vel and rad)
    """
    def _send_vel_cb(self, command):
        self._rmb.set_vel(command.vel, command.rad)

    def main(self):
        rospy.Subscriber('roomba/command',
                         VelRad, self._send_vel_cb, queue_size=1)
        rospy.Subscriber('roomba/clean',
                         Bool, self._send_clean_cb, queue_size=1)
        rospy.spin()
        self.close()


if __name__ == '__main__':
    rospy.init_node('roomba')
    rmb = RoombaIf()
    rn = RoombaNode(rmb)
    rn.main()
