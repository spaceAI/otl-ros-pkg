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
from joy.msg import Joy
from std_msgs.msg import Int32

import time
import re

# 0: 声掛け待ち
# 1: 注文受付待ち
# 2: 認識完了。確認待ち
# 3: 実行中

order_num = 0

def speak(speach):
    global talk_pub
    talk_pub.publish(String(speach))

def tray_callback(result):
    global order_state
    global order_num
    global motion_pub
    if not order_state == 1:
        return
    rospy.loginfo('order comes')
    if result.data == 1:
        order_num = 1
        #motion_pub.publish(String('sosogi1'))
        speak('ochadeyoroshiidesuka?')
    elif result.data == 2:
        order_num = 2
        #motion_pub.publish(String('sosogi2'))
        speak('akueriasudeiidesuka?')
    elif result.data == 3:
        order_num = 3
        #motion_pub.publish(String('sosogi3'))
        speak('gogonokouchadeiidesuka?')
    time.sleep(2)
    order_state = 2

def joy_callback(joy):
    global order_state
    global order_num
    if joy.buttons[8] == 1:
        if joy.buttons[12] == 1:
            speak('ochadesune')
            do_sosogi(1)
        elif joy.buttons[13] == 1:
            speak('kouchadesune')
            do_sosogi(3)
        elif joy.buttons[14] == 1:
            speak('akueriasudesune')
            do_sosogi(2)
        elif joy.buttons[4] == 1:
            if order_state == 2:
                speak('zutto orenotaan')
                do_sosogi(order_num)
        elif joy.buttons[0] == 1:
            if not order_state == 0:
              speak('dagakotowaru')
              time.sleep(5)
              order_state = 0
        elif joy.buttons[3] == 1:
            if order_state == 0:
                order_state = 1
                speak('irassyaimase')
                speak('naniwo nomitaidesuka?')

def do_sosogi(order):
    global order_state
    global cmd_pub
    if order == 0:
        return
    order_state = 3
    if order == 1:
        motion_pub.publish(String('sosogi1'))
    elif order == 2:
        motion_pub.publish(String('sosogi2'))
    elif order == 3:
        motion_pub.publish(String('sosogi3'))
    print 'motion start'
    time.sleep(20)
    print 'motion end'
    speak('yukkurishiteittene')
    time.sleep(4)
    order_state = 0

def name_callback(name):
    global order_state
    if not order_state == 0:
        return

    if (name.data):
        print 'name = ', name.data
        try:
            namae = name.data.encode().replace('_','')
            namae = re.sub("[0-9]", "", namae)
            print namae
            speak(namae+'san')
        except ValueError:
            pass
    rospy.loginfo('start talk')
    speak('irassyaimase')
    speak('naniwo nomitaidesuka?')
    order_state = 1


if __name__ == '__main__':
    global talk_pub
    global motion_pub
    global cmd_pub
    global order_state
    order_state = 0
    rospy.init_node('lv1_main', anonymous=True)
    name_sub = rospy.Subscriber('name', String, name_callback, queue_size=1)
    joy_sub  = rospy.Subscriber('joy', Joy, joy_callback, queue_size=1)
    tray_sub = rospy.Subscriber('result', Int32, tray_callback, queue_size=1)
    talk_pub = rospy.Publisher('talk', String)
    cmd_pub = rospy.Publisher('joy', Joy)
    motion_pub = rospy.Publisher('motion', String)
    counter = 0
    while not rospy.is_shutdown():
      if order_state == 0:
        counter += 1 
        if counter % 45 == 0:
            speak('ochawa ikagadesuka?')
        if counter % 45 == 15:
            speak('yukkurishiteittene')
        if counter % 45 == 30:
            speak('akueriasuwa ikagadesuka?')
      rospy.sleep(1.0)

