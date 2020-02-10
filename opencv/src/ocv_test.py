#!/usr/bin/env python
import rospy
import sys
import cv2
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
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
		ts = message_filters.ApproximateTimeSynchronizer([sub1,sub2], 1, 0.5)
		ts.registerCallback(self.image_callback)

	def image_callback(self, ros_image,camera_info):
		try:
			input_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
		except CvBridgeError, e:
			print e
		#imput_image = ros_image
		output_image = self.image_tf(input_image)
		now = rospy.Time.now()
		output_image.header.stamp = now
		camera_info.header.stamp = now
		self.image_pub.publish(output_image)
		self.info_pub.publish(camera_info)

		#cv2.imshow(self.node_name, output_image)   
		#cv2.waitKey(1)

	def image_tf(self, frame):
		#trim = frame[720/4:720-(720/4) , 1280/4:1280-(1280/4)]
		trim = frame[0:720/2 , 0:1280/2]
		#trim = frame[0:720/4 , 0:1280/4]
		result = self.bridge.cv2_to_imgmsg(trim, "bgr8")
		return result

	def cleanup(self):
		cv2.destroyAllWindows()

if __name__ == '__main__':
	camera_read()
	rospy.spin()
