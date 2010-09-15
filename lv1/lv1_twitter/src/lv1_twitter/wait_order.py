#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import with_statement
#import sys
#import codecs

import twoauth
import roslib; roslib.load_manifest('rostwitter')
import rospy
#from lv1_msgs.msg import Order

from keys import *

def order_start(data):
    global order_pub
    global last_id

    print data

    try:
        text = data[-1]['text']
        if (text.find(u'おちゃ') >= 0 or text.find(u'お茶') >= 0 or
            text.find(u'コーラ') >= 0 or text.find(u'飲み物') >= 0 or
            text.find(u'ください') >= 0 or text.find(u'下さい') >= 0
            ):
            order_msg.data = data[-1]['user']['screen_name'].encode().replace('_','')
            order_pub.publish(order_msg)
    except ValueError:
        pass

    last_id = data[-1]['id']
    with open("last_id", "w") as f:
        f.write(last_id)

if __name__ == '__main__':
    global last_id
    rospy.init_node('rostwitter', anonymous=True)
    order_pub = rospy.Publisher('name', String)

    with open("last_id") as f:
        last_id = int(f.read())

    twitter = twoauth.api(consumer_key, consumer_secret, access_token, access_token_secret)

    rate = rospy.Rate(1.0/20.0)

    while not rospy.is_shutdown():
        print 'API count = ' + twitter.rate_limit()['remaining-hits']
        data = twitter.mentions(since_id=last_id)
        if data:
            order_start(data)
        else:
            print 'no new data'
        rate.sleep()

