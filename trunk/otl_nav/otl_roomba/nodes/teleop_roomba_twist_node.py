#!/usr/bin/env python
import roslib
roslib.load_manifest('otl_roomba')

import rospy
from joy.msg import Joy
from otl_roomba.roombaif import *
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

SPEEDLIMIT = 100
EPSLIMIT = 5


def _vel2rot(vel):
    if WHEEL_OFFSET != 0:
        return vel / (WHEEL_OFFSET / 2.0)
    else:
        return 0

def _filter_limit(vel, limit):
    if vel > limit:
        vel = limit
    elif vel < -limit:
        vel = -limit
    return vel


def _egnore_eps(vel, eps):
    if abs(vel) < eps:
        vel = 0
    return vel


class RoombaTwistJoy():
    """
    control joystick by PS3 controller
    """

    def __init__(self):
        self._speed = 100.0
        self._rot = 0.0
        self._yaw_gain = -800
        self._pitch_gain = 2
    def _send_vel_cb(self, joy):
        tw = Twist()
        # L1 button 
        if joy.buttons[10] == 1:
            # pitch -->> accel
            pitch = joy.axes[17]
            self._speed = pitch * self._pitch_gain
            self._speed = _filter_limit(self._speed, SPEEDLIMIT)
            #self._speed = _egnore_eps(self._speed, EPSLIMIT)
            # yaw  -->> handle
            yaw = joy.axes[16]
            self._rot = _vel2rot(self._yaw_gain * yaw)
            self._rot = _filter_limit(self._rot, _vel2rot(SPEEDLIMIT))
            #self._rot = _egnore_eps(self._rot, _vel2rot(EPSLIMIT))
            # set twist
            tw.linear.x = self._speed
            tw.angular.z = self._rot
            #if joy.buttons[14] == 1:
            #   self._dock_pub.publish(True)
            if joy.buttons[12] == 1:
               tw.linear.x = 0
               tw.angular.z = _vel2rot(-SPEEDLIMIT)
            if joy.buttons[13] == 1:
               self._clean_pub.publish(True)
            elif joy.buttons[15] == 1:
               self._clean_pub.publish(False)

        else:
            tw.linear.x = 0
            tw.angular.z = 0
        self._twist_pub.publish(tw)

    def main(self):
        global SPEEDLIMIT
        rospy.init_node('teleop_roomba_twist')
        self._twist_pub = rospy.Publisher('roomba/command', Twist)
        self._clean_pub = rospy.Publisher('roomba/clean', Bool)
        self._dock_pub = rospy.Publisher('roomba/dock', Bool)
        rospy.Subscriber('joy', Joy, self._send_vel_cb, queue_size=1)
        SPEEDLIMIT = rospy.get_param('~max_speed', SPEEDLIMIT)
        print 'limit =', SPEEDLIMIT
        rospy.spin()

if __name__ == '__main__':
    rj = RoombaTwistJoy()
    rj.main()
