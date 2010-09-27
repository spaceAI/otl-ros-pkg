#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
roslib.load_manifest('teleop_lv1')

import rospy
from joy.msg import Joy
from std_msgs.msg import String
from joy_talk import Joy2Str

class Lv1Joy(Joy2Str):
    def Init(self):
        rospy.init_node('teleop_lv1_joy')
        self._sub = rospy.Subscriber('joy', Joy, self._joy2string, queue_size=1)
        self._pub = rospy.Publisher('motion', String)
        self._enable_button = int(rospy.get_param('~enable_button', 9))
        print 'enable is =', self._enable_button
        self._set_params('~button1', '~motion1', 4, 'reset')
        self._set_params('~button2', '~motion2', 5, 'dabadaba')
        self._set_params('~button3', '~motion3', 6, 'baibai')
        self._set_params('~button4', '~motion4', 7, 'hoge')
        self._set_params('~button5', '~motion5', 12, 'sosogi1')
        self._set_params('~button6', '~motion6', 13, 'sosogi3')
        self._set_params('~button7', '~motion7', 14, 'sosogi2')
        self._set_params('~button8', '~motion8', 15, 'yukkurishiteittene')

if __name__ == '__main__':    
    j = Lv1Joy()
    j.Init()
    rospy.spin()
