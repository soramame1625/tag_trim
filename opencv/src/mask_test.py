#!/usr/bin/env python
import rospy
import sys
import cv2
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from apriltag_ros.msg import AprilTagDetectionPositionArray
from apriltag_ros.msg import AprilTagDetectionArray
import numpy as np

import tf
from geometry_msgs.msg import Vector3

class camera_read:
	def __init__(self):
		self.node_name = "camera_trimming"
		rospy.init_node(self.node_name)
		rospy.on_shutdown(self.cleanup)
		self.bridge = CvBridge()
		self.image_pub = rospy.Publisher("/trimming_image", Image, queue_size=1)
		self.info_pub = rospy.Publisher("/trimming_info", CameraInfo, queue_size=1)
		sub1 = message_filters.Subscriber('/usb_cam/image_raw', Image)
		sub2 = message_filters.Subscriber('/usb_cam/camera_info', CameraInfo)
		self.tag_pos = rospy.Subscriber('/tag_position',AprilTagDetectionPositionArray, self.tag_pos_callback)
		self.tag_det = rospy.Subscriber('/tag_detections',AprilTagDetectionArray,self.tag_det_callback)
		ts = message_filters.ApproximateTimeSynchronizer([sub1,sub2], 1, 0.5)
		ts.registerCallback(self.image_callback)

		self.tag_pos = AprilTagDetectionPositionArray
		self.tag_pos_l = AprilTagDetectionPositionArray
		self.tag_pos_p = [0,0]
		self.xy = [0,720,0,1280]
		self.xy_l = [0,0,0,0]
		self.fg1 = 0
		self.fg2 = 0
		self.fg3 = 0

		self.tag_det = 0
		
		self.k1 = 700
		self.x = [0,0,0,0,0]
		self.x_b = [0,0,0,0,0]
		self.y = [0,0,0,0,0]
		self.y_b = [0,0,0,0,0]

		self.tag_ori = [0,0,0]
		self.color = (0,0,0)
		self.k2 = 50
		self.xy_c = [0,0,0,0]

		self.around = 0
		self.ar_x = 0
		self.ar_y = 0
		self.ar_z = 0

	def image_callback(self, ros_image,camera_info):
		try:
			input_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
		except CvBridgeError, e:
			print e
		output_image = self.image_tf(input_image)
		now = rospy.Time.now()
		output_image.header.stamp = now
		camera_info.header.stamp = now
		self.image_pub.publish(output_image)
		self.info_pub.publish(camera_info)

	def tag_pos_callback(self,data):
		#try:
		if len(data.detect_positions) >= 1:
			self.tag_pos_l = self.tag_pos
			self.tag_pos = data
			self.tag_pos_p = [self.tag_pos.detect_positions[0].x + (self.tag_pos.detect_positions[0].x - self.tag_pos_l.detect_positions[0].x), self.tag_pos.detect_positions[0].y + (self.tag_pos.detect_positions[0].y - self.tag_pos_l.detect_positions[0].y)]
			#self.tag_pos_p.detect_positions[0].y =  self.tag_pos.detect_positions[0].y + (self.tag_pos.detect_positions[0].y - self.tag_pos_l.detect_positions[0].y)

			self.around = float(self.tag_det.detections[0].size[0]) * self.k1 / float(self.tag_det.detections[0].pose.pose.pose.position.z)
			print self.around
			print self.tag_ori[2]
			if self.tag_ori[2] > 0:
				self.around = self.around * (self.tag_ori[2] + 1)
			elif self.tag_ori[2] < 0:
				self.around = self.around * ((self.tag_ori[2] * -1) + 1)
			print self.around
			self.xy = [self.tag_pos_p[1] - int(self.around), self.tag_pos_p[1] + int(self.around), self.tag_pos_p[0] - int(self.around), self.tag_pos_p[0] + int(self.around)]

			#self.xy = [self.tag_pos.detect_positions[0].y - int(self.around), self.tag_pos.detect_positions[0].y + int(self.around), self.tag_pos.detect_positions[0].x - int(self.around), self.tag_pos.detect_positions[0].x + int(self.around)]


			self.correction()
			#print self.tag_pos

		else:
			#except IndexError:
			print "tag not found"
			#self.tag_pos = 0
			self.xy = [0,720,0,1280]
			self.fg1 = 0
		self.xy_l = self.xy

	def tag_det_callback(self,data):
		#try:
		if len(data.detections) >= 1:
			self.tag_det = data
			self.tag_ori = self.q_t_e(self.tag_det.detections[0].pose.pose.pose.orientation.x,self.tag_det.detections[0].pose.pose.pose.orientation.y,self.tag_det.detections[0].pose.pose.pose.orientation.z,self.tag_det.detections[0].pose.pose.pose.orientation.w)


	def correction(self):
		self.x.append(self.xy[0])
		self.x_b.append(self.xy[1])
		self.y.append(self.xy[2])
		self.x_b.append(self.xy[3])

		self.x.pop(0)
		self.x_b.pop(0)
		self.y.pop(0)
		self.x_b.pop(0)

		if self.tag_pos.detect_positions[0].y < self.tag_pos_l.detect_positions[0].y:
			self.xy[0] = self.xy[0] - (self.tag_pos_l.detect_positions[0].y - self.tag_pos.detect_positions[0].y) * self.k2 / int(self.tag_det.detections[0].pose.pose.pose.position.z * 10)
		if self.tag_pos.detect_positions[0].y > self.tag_pos_l.detect_positions[0].y:
			self.xy[1] = self.xy[1] + (self.tag_pos.detect_positions[0].y - self.tag_pos_l.detect_positions[0].y) * self.k2 / int(self.tag_det.detections[0].pose.pose.pose.position.z * 10)
		if self.tag_pos.detect_positions[0].x < self.tag_pos_l.detect_positions[0].x:
			self.xy[2] = self.xy[2] - (self.tag_pos_l.detect_positions[0].x - self.tag_pos.detect_positions[0].x) * self.k2 / int(self.tag_det.detections[0].pose.pose.pose.position.z * 10)
		if self.tag_pos.detect_positions[0].x > self.tag_pos_l.detect_positions[0].x:
			self.xy[3] = self.xy[3] + (self.tag_pos.detect_positions[0].x - self.tag_pos_l.detect_positions[0].x) * self.k2 / int(self.tag_det.detections[0].pose.pose.pose.position.z * 10)

		print 	self.tag_det.detections[0].pose.pose.pose.position.z
		#print self.tag_pos_l.detect_positions[0].y - self.tag_pos.detect_positions[0].y

	def q_t_e(self,q_x,q_y,q_z,q_w):
		euler = tf.transformations.euler_from_quaternion((q_x, q_y, q_z, q_w))
		euler = [euler[0], euler[1], euler[2]]
		return euler

	def image_tf(self, frame):
		trim = cv2.rectangle(frame,(0,0),(1280,self.xy[0]),self.color, -1)
		trim = cv2.rectangle(frame,(0,self.xy[1]),(1280,720),self.color, -1)
		trim = cv2.rectangle(frame,(0,0),(self.xy[2],720),self.color, -1)
		trim = cv2.rectangle(frame,(self.xy[3],0),(1280,720),self.color, -1)
		#trim = cv2.putText(frame,'torasuke',(self.xy[3],self.xy[1]), cv2.FONT_HERSHEY_SIMPLEX, 4,(255,255,255),2,cv2.LINE_AA)
		result = self.bridge.cv2_to_imgmsg(trim, "bgr8")
		return result

	def cleanup(self):
		cv2.destroyAllWindows()

if __name__ == '__main__':
	camera_read()
	rospy.spin()
