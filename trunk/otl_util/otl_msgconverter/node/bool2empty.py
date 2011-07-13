#!/usr/bin/env python
import roslib
roslib.load_manifest('otl_msgconverter')

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Empty

class BoolMsg2EmpyMsg():
    """ convert bool msg to empy msg
    """
    def __init__(self, bool_topic, true_topic, false_topic):
        self._bool_topic = bool_topic
        self._true_topic = true_topic
        self._false_topic = false_topic
    def callback(self, bool_value):
        msg = Empty()
        if (bool_value.data):
            self._true_pub.publish(msg)
        else:
            self._false_pub.publish(msg)
    def init(self):
        self._true_pub = rospy.Publisher(self._true_topic, Empty)
        self._false_pub = rospy.Publisher(self._false_topic, Empty)
        self._sub = rospy.Subscriber(self._bool_topic, Bool, self.callback)

if __name__=='__main__':
    rospy.init_node('otl_bool2empty')
    obj = BoolMsg2EmpyMsg('input', 'true', 'false')
    obj.init()
    rospy.spin()
