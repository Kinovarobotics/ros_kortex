#! /bin/python3

# Tutorial link: https://dhanuzch.medium.com/using-opencv-with-gazebo-in-robot-operating-system-ros-part-1-display-real-time-video-feed-a98c078c708b
import rospy
import cv2
import numpy as np
import argparse
import imutils

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class camera_1:

  def __init__(self):
    self.image_sub = rospy.Subscriber("/my_gen3/camera/depth/image_raw", Image, self.callback)

  def callback(self,data):
    bridge = CvBridge()

    try:
      cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="32FC1")
    except CvBridgeError as e:
      rospy.logerr(e)
    
    image = cv_image.copy() #numpy array
    resized_image = cv2.resize(image, (600, 300))
    cv_image_array = np.array(resized_image, dtype = np.dtype('f8'))
    cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
    thresh = 0.09
    blurred = cv2.GaussianBlur(cv_image_norm, (5, 5), 0)
    img_bw = cv2.threshold(blurred, thresh, 255, cv2.THRESH_BINARY)[1]
    #inverted_image = cv2.bitwise_not(img_bw)
    image = cv2.resize(img_bw, (600, 300))
    image = image.astype(np.uint8)
    image = cv2.bitwise_not(image)

    cnts = cv2.findContours(image.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    print(len(cnts))
    
    for c in cnts:
      M = cv2.moments(c)
      cX = int(M["m10"] / M["m00"])
      cY = int(M["m01"] / M["m00"])
      blurred = cv2.drawContours(blurred, [c], -1, (0, 255, 0), 2)
      blurred = cv2.circle(blurred, (cX, cY), 7, (255, 255, 255), -1)
      blurred = cv2.putText(blurred, "center", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

      cv2.imshow("Camera output normal", blurred)
      #cv2.imshow("Camera output resized", image)
      cv2.waitKey(1)

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
