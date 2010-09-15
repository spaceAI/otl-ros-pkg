#!/usr/bin/env python
# -*- coding: utf-8 -*-


# メインプログラム
# 1. /name (std_msgs/String)を受け取り発話 「〜さん、いらっしゃいませ」
# 2. 移動開始 + 発話 「そちらへ移動しますのでお待ちください」
# 3. /motion (std_msgs/String)を発行
# 4. 発話「はいどうぞ」

#   - /tray (std_msgs/String)を受け取ったら発話
#   - 
from __future__ import with_statement
#import sys
#import codecs

import twoauth
import roslib; roslib.load_manifest('lv1_main')
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from lv1_msgs.msg import Order

wait_order = False

def speak(speach):
    global talk_pub
    talk_pub.publish(String(speach))

def tray_callback(result):
    global wait_order
    global motion_pub
    if not wait_order:
        return
    rospy.loginfo('order comes')
    if result.data == 0:
        motion_pub.publish(String('sosogi0'))
        speak('ochawo sosogimasu')
    elif result.data == 1:
        motion_pub.publish(String('sosogi1'))
        speak('pokariwo sosogimasu')
    elif result.data == 2:
        motion_pub.publish(String('sosogi2'))
        speak('kouchawo sosogimasu')
    wait_order = False

def name_callback(name):
    global wait_order
    if (name.data):
        print 'name = ', name.data
        try:
            namae = name.data.encode().replace('_','')
            speak(namae+'san')
        except ValueError:
            pass
    rospy.loginfo('start talk')
    speak('irassyaimase')
    speak('naniwo nomitaidesuka?')
    wait_order = True


if __name__ == '__main__':
    rospy.init_node('lv1_main', anonymous=True)
    name_sub = rospy.Subscriber('name', String, name_callback)
    tray_sub = rospy.Subscriber('result', Int32, tray_callback)
    talk_pub = rospy.Publisher('talk', String)
    motion_pub = rospy.Publisher('motion', String)
    rospy.spin()

