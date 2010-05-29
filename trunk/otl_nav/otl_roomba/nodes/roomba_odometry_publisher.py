#!/usr/bin/env python

#
# roomba odometry publisher
#
# 2010/05/21 first not tested
#
# by OTL <t.ogura@gmail.com>
#


#system
from math import cos,sin,radians, degrees
import copy
#ROS
import roslib
roslib.load_manifest('otl_roomba')
import rospy
import tf
import geometry_msgs
from nav_msgs.msg import Odometry
#OTL
from otl_roomba.roombaif import *

class RoombaOdometryPublisher:
    def __init__(self, rmb):
        self._br = tf.TransformBroadcaster()
        self._odom_pub = rospy.Publisher('odom', Odometry)
        
        self._pos = [0.0, 0.0, 0.0]
        self._diff = [0.0, 0.0, 0.0]
        
        self._rmb = rmb
        self._br = tf.TransformBroadcaster()
        self._current_time = rospy.Time.now()
        self._last_time = rospy.Time.now()
        self.vel = [0,0,0]

    def proc(self):
        self._current_time = rospy.Time.now()

        # get vel(diff) from roomba
        if not self._rmb._debug:
            self._diff[0] = self._rmb.get_distance() * 0.001
            self._diff[1] = 0
            self._diff[2] = radians(self._rmb.get_angle())
        else:
            self._diff[0] = 0.2
            self._diff[1] = 0
            self._diff[2] = 0.1

#        rospy.logerr( "diff =%f %f %f" %(self._diff[0], self._diff[1], self._diff[2]))
        # TODO
        # ignore if big num ... this is BUG may be...
        if abs(self._diff[0]) > 1 or abs(self._diff[2]) > 1:
            self._diff[0] = 0
            self._diff[2] = 0

        #compute odometry in a typical way given the velocities of the robot
        dt = (self._current_time - self._last_time).to_sec()
        delta_x = (self._diff[0] * cos(self._pos[2]) -
                   self._diff[1] * sin(self._pos[2]))
        delta_y = (self._diff[0] * sin(self._pos[2]) +
                   self._diff[1] * cos(self._pos[2]))
        delta_th = self._diff[2]

        self._pos[0] += delta_x
        self._pos[1] += delta_y
        self._pos[2] += delta_th

        #since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self._pos[2])

        rospy.logerr('pos = %f %f %f'%(self._pos[0], self._pos[1], self._pos[2]))
        #send the transform
        self._br.sendTransform((self._pos[0], self._pos[1], 0),
                               odom_quat,
                               self._current_time,
                               "base_link",
                               "odom"
                               )

        #send the transform
        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        self._br.sendTransform((0.1, 0, 0.1),
                               quat,
                               self._current_time,
                               "laser",
                               "base_link"
                               )

        #send the transform
#        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
#        self._br.sendTransform((0.0, 0, 0.0),
#                               quat,
#                               self._current_time,
#                               "odom",
#                               "map"
#                               )

        #next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = self._current_time
        odom.header.frame_id = "odom"

        #set the position
        odom.pose.pose.position.x = self._pos[0]
        odom.pose.pose.position.y = self._pos[1]
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]

        #set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = self._diff[0] / dt
        odom.twist.twist.linear.y = self._diff[1] / dt
#        odom.twist.twist.linear.x = self.vel[0]
#        odom.twist.twist.linear.y = self.vel[1]
        odom.twist.twist.linear.z = 0
        odom.twist.twist.angular.x = 0
        odom.twist.twist.angular.y = 0
        odom.twist.twist.angular.z = self._diff[2] / dt
#        odom.twist.twist.angular.z = self.vel[2]

        #publish the message
        self._odom_pub.publish(odom)

        self._last_time = self._current_time

if __name__ == '__main__':
    rospy.init_node('roomba_odometry_publisher')
    rmb = RoombaIf()
    rmb.setup()
    rop = RoombaOdometryPublisher(rmb)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rop.proc()
        rate.sleep()

