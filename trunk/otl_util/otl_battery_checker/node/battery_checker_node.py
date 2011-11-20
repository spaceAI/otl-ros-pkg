#! /usr/bin/env python

import roslib
roslib.load_manifest('otl_battery_checker')

import rospy
from std_msgs.msg import Float64
from battery_check import AcpiChecker

class BatteryCheckerNode():
    """check battery and publish the charge rate"""
    def __init__(self, battery_name='BAT0'):
        self._checker = AcpiChecker(battery_name)
        self._pub = rospy.Publisher('/pc/battery_rate', Float64)
    def proc(self):
        msg = Float64()
        msg.data = self._checker.get_rate()
        self._pub.publish(msg)

if __name__=='__main__':
    rospy.init_node('pc_battery_checker')
    battery_name = 'BAT0'
    if rospy.has_param('~device'):
        battery_name = rospy.get_param('~device')
    check = BatteryCheckerNode(battery_name)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        check.proc()
        rate.sleep()

