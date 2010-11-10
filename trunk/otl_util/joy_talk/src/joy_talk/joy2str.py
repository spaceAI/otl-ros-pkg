#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
roslib.load_manifest('joy_talk')

import rospy
from joy.msg import Joy
from std_msgs.msg import String

class Joy2Str():
    """ joy -> string """
    def __init__(self):
        self._last_index = -1
        self._button_string = {}
        self._enable_button = 0

    def _joy2string(self, joy):
        if (self._enable_button < len(joy.buttons) and
            joy.buttons[self._enable_button] == 1):
            for index in range(len(joy.buttons)):
                if (index < len(joy.buttons) and
                    joy.buttons[index] == 1):
                    if (index in self._button_string):
                        str = self._button_string[index]
                        if (str and (not self._last_index == index)):
                            print 'pub!', str
                            self._pub.publish(String(str))
                            self._last_index = index
                # button == 0 かつ前回それを押してたとき
                elif(self._last_index == index):
                    self._last_index = -1

    def _set_params(self, button, string, default_button, default_string):
        button_param = rospy.get_param(button, default_button)
        string_param = rospy.get_param(string, default_string)
        if (button_param and string_param):
            self._button_string[int(button_param)] = string_param
            print 'setting ', button_param , ' = ', string_param

