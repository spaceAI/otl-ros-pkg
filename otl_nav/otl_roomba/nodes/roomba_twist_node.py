#!/usr/bin/env python
import roslib
roslib.load_manifest('otl_roomba')

import rospy
from otl_roomba.roombaif import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from roomba_base_node import RoombaBaseNode
from roomba_odometry_publisher import RoombaOdometryPublisher

class RoombaTwistNode(RoombaBaseNode):
    """
    roomba interface for ROS (twist vel interface)
    """
    def _send_vel_cb(self, twist):
        self._rmb.set_twist_vel(twist.linear.x * 1000, twist.angular.z)
        self._odom.vel = [twist.linear.x, 0, twist.angular.z]
#        self._odom.proc()
#        if ( self._rmb.get_voltage() < 12 * 1000):
#            self._rmb.play_song(0)
    def _activate(self, empty):
        self._rmb.activate()

    def main(self):
        rospy.Subscriber('roomba/command',
                         Twist, self._send_vel_cb, queue_size=1)
        rospy.Subscriber('roomba/clean',
                         Bool, self._send_clean_cb, queue_size=1)
        rospy.Service('activate', Empty, self._activate)

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self._odom.proc()
            rate.sleep()
        self.close()

    def set_odometry(self, rop):
        self._odom = rop

if __name__ == '__main__':
    rmb = RoombaIf()
    rn = RoombaTwistNode(rmb)
    rop = RoombaOdometryPublisher(rmb)
    rn.set_odometry(rop)
    rn.main()
