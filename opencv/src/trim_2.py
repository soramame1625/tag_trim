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
		#self.tag_sub = rospy.Subscriber('/tag_position',AprilTagDetectionPositionArray, self.tag_callback)
		ts = message_filters.ApproximateTimeSynchronizer([sub1,sub2], 1, 0.5)
		ts.registerCallback(self.image_callback)

		self.tag_pos = 0
		self.tag_pos_l = 0
		self.xy = [0,720,0,1280]
		self.xy_l = [0,0,0,0]
		self.fg1 = 0
		self.fg2 = 0
		self.fg3 = 0

		self.x = [0,0,0,0,0]
		self.x_b = [0,0,0,0,0]
		self.y = [0,0,0,0,0]
		self.y_b = [0,0,0,0,0]

		self.color = (0,0,0)
		self.conf = 2
		self.xy_c = [0,0,0,0]

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

	def tag_callback(self,data):
		#try:
		if len(data.detect_positions) >= 1:
			self.tag_pos = data
			self.xy = [self.tag_pos.detect_positions[0].y - 50, self.tag_pos.detect_positions[0].y + 50, self.tag_pos.detect_positions[0].x - 50, self.tag_pos.detect_positions[0].x + 50]

			#print self.tag_pos

		else:
			#except IndexError:
			print "tag not found"
			#self.tag_pos = 0
			self.xy = [0,720,0,1280]
			self.fg1 = 0
		self.tag_pos_l = self.tag_pos
		self.xy_l = self.xy

	def image_tf(self, frame):
		#self.xy = [0,70,0,70]
		self.xy = [720/4,720-(720/4) , 1280/4,1280-(1280/4)]
		#self.xy = [0,720/16,0,1280/32]
		trim = cv2.rectangle(frame,(0,0),(1280,self.xy[0]),self.color, -1)
		trim = cv2.rectangle(frame,(0,self.xy[1]),(1280,720),self.color, -1)
		trim = cv2.rectangle(frame,(0,0),(self.xy[2],720),self.color, -1)
		trim = cv2.rectangle(frame,(self.xy[3],0),(1280,720),self.color, -1)
		#trim = cv2.putText(frame,'torasuke',(self.xy[3],self.xy[1]), cv2.FONT_HERSHEY_SIMPLEX, 4,(255,255,255),2,cv2.LINE_AA)
		#trim = cv2.putText(fram, 'aaaaaaaa', (self.tag_pos.detect_positions[0].x, self.tag_pos.detect_positions[0].y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)
		result = self.bridge.cv2_to_imgmsg(trim, "bgr8")
		return result

	def cleanup(self):
		cv2.destroyAllWindows()

if __name__ == '__main__':
	camera_read()
	rospy.spin()
