#!/usr/bin/env python

import twoauth
import roslib; roslib.load_manifest('rostwitter')
import rospy
from std_msgs.msg import String

consumer_key = 'feliLoH9Zlp6hfxO8jhEA'
consumer_secret = 'OtVjrXVezSMxVJrf99HlfTr2fsHQxcEJaAPdxfStkA'
access_token = '4416471-cKU7HJPDnQt5yAZ98zbhSertqhSyLFSnntX1EQhbaw'
access_token_secret = 'S68gl5qK11gIYAhRSnkDLJQQUy4I2Goei3tUVg5F0E'

def twit(data):
    rospy.loginfo(rospy.get_name()+"I heard %s",data.data)
    twitter.status_update(data.data)

def listener():
    rospy.init_node('rostwit_server', anonymous=True)
    rospy.Subscriber("twit", String, twit)
    rospy.spin()

if __name__ == '__main__':
    twitter = twoauth.api(consumer_key, consumer_secret, access_token, access_token_secret)
    listener()


