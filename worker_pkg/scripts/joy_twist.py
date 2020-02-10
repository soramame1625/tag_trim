#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyTwist(object):
    def __init__(self):
        self._max_vel_mm = 250
        self._max_vel_deg = 30
        self._joy_sub =\
        rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=1)
        self._twist_pub_0 =\
        rospy.Publisher('/owmr_0/vxyw_in', Twist, queue_size=1)
        self._twist_pub_1 =\
        rospy.Publisher('/owmr_1/vxyw_in', Twist, queue_size=1)
        self._twist_pub_2 =\
        rospy.Publisher('/owmr_2/vxyw_in', Twist, queue_size=1)
        self._twist_pub_3 =\
        rospy.Publisher('/owmr_3/vxyw_in', Twist, queue_size=1)

    def joy_callback(self, joy_msg):
        twist = Twist()
        twist.linear.x = joy_msg.axes[1] * self._max_vel_mm
        twist.linear.y = joy_msg.axes[0] * self._max_vel_mm
        twist.angular.z = joy_msg.axes[2] * self._max_vel_deg
        self._twist_pub_0.publish(twist)
        self._twist_pub_1.publish(twist)
        self._twist_pub_2.publish(twist)
        self._twist_pub_3.publish(twist)

if __name__ == '__main__':
    rospy.init_node('joy_twist')
    joy_twist = JoyTwist()
    rospy.spin()
