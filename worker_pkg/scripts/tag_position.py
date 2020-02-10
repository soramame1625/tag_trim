#!/usr/bin/env python
import rospy
from apriltags2_ros.msg import AprilTagDetectionArray
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int16
import numpy as np

import tf
import geometry_msgs.msg
import math

class Tag_taker(object):
    def __init__(self):
        rospy.init_node('tag_position')
        self._ap_data = np.zeros((4,7), dtype = 'float64')
        self.position_data = np.zeros((4,6), dtype = 'float64')
        self.cm_data = np.zeros(7, dtype = 'float64')
        self.tags_state = 0
        self.listener = tf.TransformListener()

        self._tag1_pub = rospy.Publisher('/tag1_point', Float64MultiArray, queue_size=10)
        self._tag2_pub = rospy.Publisher('/tag2_point', Float64MultiArray, queue_size=10)
        self._tag3_pub = rospy.Publisher('/tag3_point', Float64MultiArray, queue_size=10)
        self._cam_pub = rospy.Publisher('/cam_point', Float64MultiArray, queue_size=10)
        self._state_pub = rospy.Publisher('/tags_state', Int16, queue_size=10)
        self.sub = rospy.Subscriber('/tag_detections',AprilTagDetectionArray,self.callback)
        rospy.spin()

    def get_pose(self,p_list, name):
        br = tf.TransformBroadcaster()
        br.sendTransform(
            p_list[0:3],
            p_list[3:7],
            rospy.Time.now(),
            name,
            'usb_cam')

    def change_angle(self,quaternion):
        #External parameters
        e = tf.transformations.euler_from_quaternion(quaternion)    #change angle
        return e

    def publish_tag_1(self,list_data):
        position_list = Float64MultiArray()
        position_list.data = list_data
        self._tag1_pub.publish(position_list)

    def publish_tag_2(self,list_data):
        position_list = Float64MultiArray()
        position_list.data = list_data
        self._tag2_pub.publish(position_list)

    def publish_tag_3(self,list_data):
        position_list = Float64MultiArray()
        position_list.data = list_data
        self._tag3_pub.publish(position_list)

    def publish_cam(self,list_data):
        position_list = Float64MultiArray()
        position_list.data = list_data
        self._cam_pub.publish(position_list)

    def publish_tags_state(self,data):
        state = Int16()
        state.data = data
        self._state_pub.publish(state)

    def callback(self,messege):
        tags = len(messege.detections)
        #print tags
        if tags > 0:
            for i in xrange(tags):
                for j in xrange(4):
                    if messege.detections[i].id[0] == 0:
                        self.tags_state = 1
                    if messege.detections[i].id[0] == j:
                        self._ap_data[j][0] = messege.detections[i].pose.pose.pose.position.x
                        self._ap_data[j][1] = messege.detections[i].pose.pose.pose.position.y
                        self._ap_data[j][2] = messege.detections[i].pose.pose.pose.position.z
                        self._ap_data[j][3] = messege.detections[i].pose.pose.pose.orientation.x
                        self._ap_data[j][4] = messege.detections[i].pose.pose.pose.orientation.y
                        self._ap_data[j][5] = messege.detections[i].pose.pose.pose.orientation.z
                        self._ap_data[j][6] = messege.detections[i].pose.pose.pose.orientation.w
        if self.tags_state == 1:
            self.get_pose(self._ap_data[0],"base")
            print self._ap_data[0]

            try:
                (trans1,rot1) = self.listener.lookupTransform('base', 'usb_cam', rospy.Time(0))
                self.camera_data = trans1 + list(self.change_angle(rot1))
                print 'cm:0 position',self.camera_data[0:3]
                print 'cm:0 angle',math.degrees(self.camera_data[3])
                print 'cm:0 angle',math.degrees(self.camera_data[4])
                print 'cm:0 angle',math.degrees(self.camera_data[5])
                self.publish_cam(self.camera_data)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

            try:
                (trans0,rot0) = self.listener.lookupTransform('base', 'tag1', rospy.Time(0))
                self.position_data[0] = trans0 + list(self.change_angle(rot0))
                print 'ID:1 position ',self.position_data[0][0:3]
                print 'ID:1 theta    ',math.degrees(self.position_data[0][5])
                self.publish_tag_1(self.position_data[0])
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

            try:
                (trans0,rot0) = self.listener.lookupTransform('base', 'tag2', rospy.Time(0))
                self.position_data[1] = trans0 + list(self.change_angle(rot0))
                print 'ID:2 position ',self.position_data[1][0:3]
                print 'ID:2 theta    ',math.degrees(self.position_data[1][5])
                self.publish_tag_2(self.position_data[1])
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

            try:
                (trans0,rot0) = self.listener.lookupTransform('base', 'tag3', rospy.Time(0))
                self.position_data[2] = trans0 + list(self.change_angle(rot0))
                print 'ID:3 position ',self.position_data[2][0:3]
                print 'ID:3 theta    ',math.degrees(self.position_data[2][5])
                self.publish_tag_3(self.position_data[2])
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

            self.publish_tags_state(self.tags_state)

if __name__ == "__main__":
    tag_taker = Tag_taker()

