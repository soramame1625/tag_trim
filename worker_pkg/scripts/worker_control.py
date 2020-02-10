#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class Worker_control(object):
    def __init__(self):
        self._target_x = 0
        self._target_y = 0
        self._target_theta = 0
        self._grobal_x = 0
        self._grobal_y = 0
        self._grobal_theta = 0

        self._max_vel_mm = 250
        self._max_vel_deg = 30
        self._joy_sub =\
        rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=1)
        self._tag_sub =\
        rospy.Subscriber('/tag3_point',Float64MultiArray,self.tag_callback, queue_size=1)
        self._twist_pub_0 =\
        rospy.Publisher('/owmr_0/vxyw_in', Twist, queue_size=1)
        self._twist_pub_1 =\
        rospy.Publisher('/owmr_1/vxyw_in', Twist, queue_size=1)
        self._twist_pub_2 =\
        rospy.Publisher('/owmr_2/vxyw_in', Twist, queue_size=1)
        self._twist_pub_3 =\
        rospy.Publisher('/owmr_3/vxyw_in', Twist, queue_size=1)

    def joy_callback(self, joy_msg):
        self._target_x = 500*joy_msg.axes[1]
        self._target_y = 500*joy_msg.axes[0]

    def tag_callback(self, tag_msg):
        twist = Twist()
        self._grobal_x = tag_msg.data[0]
        self._grobal_y = tag_msg.data[1]
        self._grobal_theta = tag_msg.data[5]

    def position_control(self):
        twist = Twist()
        kp_theta = 1.2
        kp_xy = 1
        e_x = self._target_x - (1000*self._grobal_x)
        e_y = self._target_y - (1000*self._grobal_y)
        e_theta = self._target_theta - math.degrees(self._grobal_theta)

        twist.linear.x = kp_xy * e_y * math.cos(self._grobal_theta)
        if twist.linear.x > self._max_vel_mm:
            twist.linear.x = self._max_vel_mm
        elif twist.linear.x < -self._max_vel_mm:
            twist.linear.x = -self._max_vel_mm

        twist.linear.y = -kp_xy * e_x * math.cos(self._grobal_theta)
        # twist.linear.y = kp_xy * e_x * (-math.cos(self._grobal_theta))
        if twist.linear.y > self._max_vel_mm:
            twist.linear.y = self._max_vel_mm
        elif twist.linear.y < -self._max_vel_mm:
            twist.linear.y = -self._max_vel_mm

        twist.angular.z = kp_theta * e_theta
        if twist.angular.z > self._max_vel_deg:
            twist.angular.z = self._max_vel_deg
        elif twist.angular.z < -self._max_vel_deg:
            twist.angular.z = -self._max_vel_deg
        print e_x,e_y
        print twist.angular.z

        self._twist_pub_0.publish(twist)
        self._twist_pub_1.publish(twist)
        self._twist_pub_2.publish(twist)
        self._twist_pub_3.publish(twist)

    def main(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.position_control()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('worker_control')
    worker_control = Worker_control()
    worker_control.main()
