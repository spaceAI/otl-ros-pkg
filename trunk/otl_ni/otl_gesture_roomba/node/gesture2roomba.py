#!/usr/bin/env python

import roslib
roslib.load_manifest('otl_gesture_roomba')

import rospy

from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class Gesture2Roomba():
    """ Convert gesture msgs to roomba commands
    """
    def __init__(self):
        self._speed = 0.3
        self._rot = 1.0
    def _gesture_callback(self, gesture):
        tw = Twist()
        if gesture.data == "forward":
            tw.linear.x = self._speed
            self._twist_pub.publish(tw)
        elif gesture.data == "backward":
            tw.linear.x = -self._speed
            self._twist_pub.publish(tw)
        elif gesture.data == "right":
            tw.angular.z = -self._rot
            self._twist_pub.publish(tw)
        elif gesture.data == "left":
            tw.angular.z = self._rot
            self._twist_pub.publish(tw)
        if gesture.data == "on":
            clean = Bool()
            clean.data = True
            self._clean_pub.publish(clean)
            self._twist_pub.publish(tw)
        elif gesture.data == "off":
            clean = Bool()
            clean.data = False
            self._clean_pub.publish(clean)
            self._twist_pub.publish(tw)
    def main(self):
        rospy.init_node("gesture2roomba")
        self._sub = rospy.Subscriber('gesture', String, self._gesture_callback)
        self._twist_pub = rospy.Publisher('roomba/command', Twist)
        self._clean_pub = rospy.Publisher('roomba/clean', Bool)
        rospy.spin()

if __name__ == '__main__':
    gesture2roomba = Gesture2Roomba()
    gesture2roomba.main()
