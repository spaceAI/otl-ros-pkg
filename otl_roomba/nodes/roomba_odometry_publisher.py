#!/usr/bin/env python
import roslib
roslib.load_manifest('otl_roomba')

import rospy
from otl_roomba.roombaif import *
import tf
import geometry_msgs
from nav_msgs.msg import Odometry
from math import cos,sin

class RoombaOdometryPublisher:
    def __init__(self, rmb):
        self._br = tf.TransformBroadcaster()
        self._odom_pub = rospy.Publisher('odom', Odometry)
        
        self._pos = [0.0, 0.0, 0.0]
        self._vel = [0.0, 0.0, 0.0]
        
        self._rmb = rmb
        self._br = tf.TransformBroadcaster()
        self._current_time = rospy.Time.now()
        self._last_time = rospy.Time.now()

    def proc(self):
        self._current_time = rospy.Time.now()

        # get vel(diff) from roomba
        if not self._rmb._debug:
            self._vel[0] = self._rmb.get_distance()
            self._vel[1] = 0
            self._vel[2] = self._rmb.get_angle()
        else:
            self._vel[0] = 0.1
            self._vel[1] = 0
            self._vel[2] = 0.1
            
        #compute odometry in a typical way given the velocities of the robot
        dt = (self._current_time - self._last_time).to_sec()
        delta_x = (self._vel[0] * cos(self._pos[2]) -
                   self._vel[1] * sin(self._pos[2])) * dt
        delta_y = (self._vel[0] * sin(self._pos[2]) +
                   self._vel[1] * cos(self._pos[2])) * dt
        delta_th = self._vel[2] * dt
        
        self._pos[0] += delta_x
        self._pos[1] += delta_y
        self._pos[2] += delta_th

        #since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self._pos[2])
        

        #send the transform
        self._br.sendTransform((self._pos[0], self._pos[1], 0),
                               odom_quat,
                               self._current_time,
                               "base_link",
                               "odom"
                               )

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
        odom.twist.twist.linear.x = self._vel[0]
        odom.twist.twist.linear.y = self._vel[1]
        odom.twist.twist.linear.z = 0
        odom.twist.twist.angular.x = 0
        odom.twist.twist.angular.y = 0
        odom.twist.twist.angular.z = self._vel[2]

        #publish the message
        self._odom_pub.publish(odom)

        self._last_time = self._current_time


if __name__ == '__main__':
    rospy.init_node('roomba_odometry_publisher')
    rmb = RoombaIf(debug=True)
    rop = RoombaOdometryPublisher(rmb)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rop.proc()
        rate.sleep()

