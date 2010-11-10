#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
roslib.load_manifest('joy_talk')

import rospy
from joy.msg import Joy
from joy2str import Joy2Str
from std_msgs.msg import String

class JoyTalk(Joy2Str):
    def Init(self):
        rospy.init_node('joy_talk')
        self._sub = rospy.Subscriber('joy', Joy, self._joy2string, queue_size=1)
        self._pub = rospy.Publisher('talk', String)
        self._enable_button = int(rospy.get_param('~enable_button', 11))
        print 'enable is =', self._enable_button
        self._set_params('~button1', '~string1', 4, 'konnnichiwa')
        self._set_params('~button2', '~string2', 5, 'irassyaimase')
        self._set_params('~button3', '~string3', 6, 'arigatougozaimasu')
        self._set_params('~button4', '~string4', 7, 'matakitene')
        self._set_params('~button5', '~string5', 12, 'gomennnasai')
        self._set_params('~button6', '~string6', 13, 'ochawaikagadesuka?')
        self._set_params('~button7', '~string7', 14, 'nodowakawakimasenka?')
        self._set_params('~button8', '~string8', 15, 'yukkurishiteittene')

if __name__ == '__main__':    
    j = JoyTalk()
    j.Init()
    rospy.spin()
