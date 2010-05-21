#!/usr/bin/env python
import roslib
roslib.load_manifest('otl_roomba')

import rospy
from joy.msg import Joy
from otl_roomba.roombaif import *
from otl_roomba.msg import VelRad
from std_msgs.msg import Bool


SPEEDLIMIT = 400


class RoombaJoy:
    """
    control joystick by PS3 controller
    """

    def __init__(self):
        self._speed = 100
        self._yaw_gain = 20000
        self._pitch_gain = 200
        self._dv = 10
    def _send_vel_cb(self, joy):
        v = VelRad()
        if joy.buttons[12] == 1:
            self._speed += self._dv
        elif joy.buttons[14] == 1:
            self._speed -= self._dv
        if joy.buttons[4] == 1:
            pitch = joy.axes[17]
            self._speed += pitch * self._pitch_gain
            if self._speed > SPEEDLIMIT:
                self._speed = SPEEDLIMIT
            elif self._speed < -SPEEDLIMIT:
                self._speed = -SPEEDLIMIT
            yaw = joy.axes[16]
            if (yaw > 0):
                r = self._yaw_gain * yaw - MAXRAD
                if r >= 0:
                    r = -1
            else:
                r = self._yaw_gain * yaw + MAXRAD
                if r <= 0:
                    r = 1
#            print "%f %d" % (yaw, r)
#            self.rmb_.setVel(self._speed, r)
            v.vel = self._speed
            v.rad = r
        elif joy.buttons[6] == 1:
            v.vel = -self._speed
            v.rad = STRAIGHTRAD
        elif joy.buttons[5] == 1:
            v.vel = self._speed
            v.rad = CWRAD
        elif joy.buttons[7] == 1:
            v.vel = self._speed
            v.rad = CCWRAD
        else:
            v.vel = 0
            v.rad = STRAIGHTRAD
        self._vel_pub.publish(v)

        if joy.buttons[13] == 1:
            self._clean_pub.publish(True)
        elif joy.buttons[15] == 1:
            self._clean_pub.publish(False)

    def main(self):
        rospy.init_node('teleop_roomba')
        self._vel_pub = rospy.Publisher('roomba/command', VelRad)
        self._clean_pub = rospy.Publisher('roomba/clean', Bool)
        rospy.Subscriber('joy', Joy, self._send_vel_cb, queue_size=1)

        rospy.spin()

if __name__ == '__main__':
    rj = RoombaJoy()
    rj.main()
