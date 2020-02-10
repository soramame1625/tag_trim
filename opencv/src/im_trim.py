#!/usr/bin/env python
import rospy
import sys
import cv2
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from apriltag_ros.msg import AprilTagDetectionPositionArray
import numpy as np

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
		self.tag_sub = rospy.Subscriber('/tag_position',AprilTagDetectionPositionArray, self.tag_callback)
		ts = message_filters.ApproximateTimeSynchronizer([sub1,sub2], 1, 0.5)
		ts.registerCallback(self.image_callback)

		self.tag_pos = 0
		self.tag_pos_l = 0
		self.xy = [0,720,0,1280]
		self.xy_l = [0,0,0,0]
		self.fg1 = 0
		self.fg2 = 0
		self.fg3 = 0

	def image_callback(self, ros_image,camera_info):
		try:
			input_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
		except CvBridgeError, e:
			print e
		imput_image = ros_image
		output_image = self.image_tf(input_image)
		now = rospy.Time.now()
		output_image.header.stamp = now
		camera_info.header.stamp = now
		self.image_pub.publish(output_image)
		self.info_pub.publish(camera_info)
		#cv2.imshow(self.node_name, output_image)   
		#cv2.waitKey(1)

	def tag_callback(self,data):
		#try:
		if len(data.detect_positions) >= 1:

			self.tag_pos = data
			self.tag_pos.detect_positions[0].x = self.tag_pos.detect_positions[0].x + self.xy[2]
			self.tag_pos.detect_positions[0].y = self.tag_pos.detect_positions[0].y + self.xy[0]
			print self.xy
			self.xy_l = self.xy
			self.xy = [self.tag_pos.detect_positions[0].y - 50, self.tag_pos.detect_positions[0].y + 50, self.tag_pos.detect_positions[0].x - 50, self.tag_pos.detect_positions[0].x + 50]

			#if self.tag_pos_l == 0:

			#self.tag_pos.detect_positions[0].x = self.tag_pos.detect_positions[0].x + self.xy[2]
			#self.tag_pos.detect_positions[0].y = self.tag_pos.detect_positions[0].y + self.xy[0]

			
			#if self.fg1 == 0:
			#self.xy = [self.tag_pos.detect_positions[0].y - 50, self.tag_pos.detect_positions[0].y + 50, self.tag_pos.detect_positions[0].x - 50, self.tag_pos.detect_positions[0].x + 50]


			
			if self.xy[0] < 0:
				self.xy[0] = 0
			if self.xy[1] > 720:
				self.xy[1] = 720
			if self.xy[2] < 0:
				self.xy[2] = 0
			if self.xy[3] > 1280:
				self.xy[3] = 1280

			#print self.tag_pos

		else:
			#except IndexError:
			print "tag not found"
			#self.tag_pos = 0
			self.xy = [0,720,0,1280]
			self.fg1 = 0
			#self.fg2 = 0
		#self.tag_pos.detect_positions[0].x = self.tag_pos.detect_positions[0].x + self.xy[2]
		#self.tag_pos.detect_positions[0].y = self.tag_pos.detect_positions[0].y + self.xy[0]
		self.tag_pos_l = self.tag_pos
		#self.xy_l = self.xy

	def image_tf(self, frame):
		trim = frame[720/4:720-(720/4) , 1280/4:1280-(1280/4)]
		#trim = frame[self.xy[0]:self.xy[1] , self.xy[2]:self.xy[3]]
		#trim = frame[0:720 , 0:1280]
		#trim = frame[0:30 , 0:30]
		result = self.bridge.cv2_to_imgmsg(trim, "bgr8")
		return result

	def cleanup(self):
		cv2.destroyAllWindows()

if __name__ == '__main__':
	camera_read()
	rospy.spin()
