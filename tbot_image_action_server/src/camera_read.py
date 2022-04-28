#! /bin/python3

# Tutorial link: https://dhanuzch.medium.com/using-opencv-with-gazebo-in-robot-operating-system-ros-part-1-display-real-time-video-feed-a98c078c708b
from unittest import result
import rospy
import cv2
import numpy as np
import argparse
import imutils

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class camera_1:

  def __init__(self):
    self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

  def callback(self,data):
    bridge = CvBridge()

    try:
      cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)
    
    image = cv_image.copy() #numpy array
    resized_image = cv2.resize(image, (600, 300))
  
    cv2.imshow("Camera output normal", resized_image)
      #cv2.imshow("Camera output resized", image)
    cv2.waitKey(0)

def main():
	camera_1()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down")
	
	cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('camera_read', anonymous=False)
    main()
