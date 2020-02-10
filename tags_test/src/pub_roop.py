#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
import math
import tf
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray

class Tag_listener(object):
  def __init__(self):
    self._sub_tag = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self._callback_tag)

    self._base_pub = rospy.Publisher("base_position", Twist, queue_size = 10)#基準用に置くTag0の座標
    self._cam_pub = rospy.Publisher("camera_position", Twist, queue_size = 10)
    self._machine1_pub = rospy.Publisher("zuo_position", Twist, queue_size = 10)
    self._machine2_pub = rospy.Publisher("bami_position", Twist, queue_size = 10)

    self._tag_position = np.zeros(4, dtype = "float64")
    self._base_position = np.zeros(7, dtype = "float64")
    self._base_twist = Twist()
    self._world_position = np.zeros(7, dtype = "float64")
    self._cam_position = Twist()

    self._zuo_position = Twist()
    self._bami_position = Twist()

  def _get_pose(self, p_list, name, base_name):
    br = tf.TransformBroadcaster()
    br.sendTransform(p_list[0:3],
      p_list[3:7],
      rospy.Time.now(),
      name,
      base_name)

  def main(self):
    rospy.init_node("tag_listener")
    self._listener = tf.TransformListener()

    if not self._base_twist.linear.x and not self._cam_position.linear.x:
      pass
    else:
      self._cam_pub.publish(self._cam_position)
      self._base_pub.publish(self._base_twist)

    rospy.spin()

  def _callback_tag(self, message):
    if len(message.detections) > 0:
      for i in range(len(message.detections)):
        angle_q = message.detections[i].pose.pose.pose.orientation
        angle_r = self._change_angle([angle_q.x,angle_q.y,angle_q.z,angle_q.w])

        self._tag_id = message.detections[i].id[0]
        self._tag_position[0] = message.detections[i].pose.pose.pose.position.x
        self._tag_position[1] = message.detections[i].pose.pose.pose.position.y
        self._tag_position[2] = message.detections[i].pose.pose.pose.position.z
        self._tag_position[3] = math.degrees(angle_r[2])

        try:
          if self._tag_id == 0:
            self._base_position = [self._tag_position[0],self._tag_position[1],self._tag_position[2],
            angle_q.x, angle_q.y, angle_q.z, angle_q.w]
            
            print "base_tag_recorded"


            (trans_cam, rot) = self._listener.lookupTransform("base", "usb_cam", rospy.Time(0))
            self._base_twist.linear.x = trans_cam[0]
            self._base_twist.linear.y = trans_cam[1]
            self._base_twist.linear.z = trans_cam[2]
            self._base_twist.angular.x = angle_r[0]
            self._base_twist.angular.y = angle_r[1]
            self._base_twist.angular.z = self._tag_position[3]

          self._get_pose(self._base_position,"base","usb_cam")
          self._get_pose([-0.805, -0.80, 0.0, 0, 0, 0, 1], "world", "base")
          (trans_cam, rot_cam) = self._listener.lookupTransform("world", "usb_cam", rospy.Time(0))
          self._cam_position.linear.x = trans_cam[0]
          self._cam_position.linear.y = trans_cam[1]
          self._cam_position.linear.z = trans_cam[2]
          self._cam_position.angular.x = rot_cam[0]
          self._cam_position.angular.y = rot_cam[1]
          self._cam_position.angular.z = rot_cam[2] 

          if self._tag_id == 1:
            (trans, rot) = self._listener.lookupTransform("world", "tag1", rospy.Time(0))
            rot = self._change_angle(rot)
            self._zuo_position.linear.x = trans[0]
            self._zuo_position.linear.y = trans[1]
            self._zuo_position.linear.z = trans[2]
            self._zuo_position.angular.z = math.degrees(rot[2])
            self._machine1_pub.publish(self._zuo_position)

            self._print_position(trans, math.degrees(rot[2]), 1)

          elif self._tag_id == 2:
            (trans, rot) = self._listener.lookupTransform("world", "tag2", rospy.Time(0))
            rot = self._change_angle(rot)
            self._bami_position.linear.x = trans[0]
            self._bami_position.linear.y = trans[1]
            self._bami_position.linear.z = trans[2]
            self._bami_position.angular.z = math.degrees(rot[2])
            self._machine2_pub.publish(self._bami_position)

            self._print_position(trans, math.degrees(rot[2]), 2)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          pass
    else:
      print "nothing"
    if not self._base_position[0]:
      print "Please put base_tag"
#    else:
#      self._base_pub.publish(self._base_twist)
#      self._cam_pub.publish(self._cam_position)

  def _change_angle(self,quaternion):
    #External parameters
    e = tf.transformations.euler_from_quaternion(quaternion)    #change angle
    return e

  def _print_position(self, tag_data, tag_theta, tag_id):
    if tag_theta <= 0.0:
      tag_theta = 360.0 + tag_theta

if __name__ == "__main__":
    tag_listener = Tag_listener()
    tag_listener.main()
