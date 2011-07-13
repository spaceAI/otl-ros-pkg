#!/usr/bin/env python
import roslib
roslib.load_manifest('otl_msgconverter')
import sys
sys.path.append('../node')
import bool2empty

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Empty
import unittest
import time

class TestMsg(unittest.TestCase):
    def setUp(self):
        rospy.init_node('otl_bool2empty_unitest')
        self._obj = bool2empty.BoolMsg2EmpyMsg('input', 'true', 'false')
        self._obj.init()

        self._true = False
        self._false = False
        self._true_sub = rospy.Subscriber('true', Empty, self.true_callback)
        self._false_sub = rospy.Subscriber('false', Empty, self.false_callback)
        self._pub = rospy.Publisher('input', Bool)
    def true_callback(self, msg):
        self._true = True
        self._false = False
    def false_callback(self, msg):
        self._true = False
        self._false = True
    def test1(self):
        msg = Bool()
        msg.data = True
        # wait until advertise and receive
        time.sleep(0.5)
        self._pub.publish(msg)
        time.sleep(0.1)
        self.assertTrue(self._true)
        self.assertFalse(self._false)

        msg.data = False
        self._pub.publish(msg)
        time.sleep(0.1)
        self.assertTrue(self._false)
        self.assertFalse(self._true)

if __name__ == '__main__':
    unittest.main()
