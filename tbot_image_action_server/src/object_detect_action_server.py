#! /bin/python3
from cmath import rect
from enum import unique
from tracemalloc import start
from unittest import result
import rospy
import time
import actionlib
from tbot_image_action_server.msg import ObjectDetectActionMsgGoal, ObjectDetectActionMsgResult, ObjectDetectActionMsgFeedback, ObjectDetectActionMsgAction
from actionlib.msg import TestFeedback, TestResult, TestAction
from geometry_msgs.msg import Twist
import cv2
from sensor_msgs.msg import Image
import numpy as np
import imutils
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class object_finder(object):
  # create messages that are used to publish feedback/result
  _feedback = ObjectDetectActionMsgFeedback()
  _result = ObjectDetectActionMsgResult()
  _success = False
  # decoder varriables
  _bridge = CvBridge()
  _thresh = 0.1

  def __init__(self):
    # creates the action server
    self._as = actionlib.SimpleActionServer("object_detect_as", ObjectDetectActionMsgAction, self.goal_callback, False)
    self._as.start()
    self.ctrl_c = False
    self._feedback.feedback = "found: 0 objects"
    print("started server")

  def callback(self,data):
    print("recived request")
    try:
      cv_image = self._bridge.imgmsg_to_cv2(data, desired_encoding="32FC1")
    except CvBridgeError as e:
      rospy.logerr(e)

    image = cv_image.copy() #numpy array
    
    #resized_image = cv2.resize(image, (600, 300))
    cv_image_array = np.array(image, dtype = np.dtype('f8'))
    cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
    blurred = cv2.GaussianBlur(cv_image_norm, (5, 5), 0)
    img_bw = cv2.threshold(blurred, self._thresh, 255, cv2.THRESH_BINARY)[1]
    #image = cv2.resize(img_bw, (600, 300))
    color_image = img_bw.astype(np.uint8)
    negative_img = cv2.bitwise_not(color_image)
    print(str(negative_img.shape))
    negative_img = negative_img[20:240,80:400]

    cnts = cv2.findContours(negative_img.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    for c in cnts:
      M = cv2.moments(c)
      if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"]) # find center x cord
        cY = int(M["m01"] / M["m00"]) # find center y cord
        cv2.drawContours(blurred, [c], -10, (255, 255, 255), 3)
        cv2.circle(blurred, (cX, cY), 7, (255, 255, 255), -1)
        cv2.putText(blurred, "center", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    if len(cnts) >= self._result.result:
        self._feedback.feedback = "found: " + str(len(cnts)) + " objects"
        self._result.result = len(cnts)
        cv2.imshow("Camera output normal", blurred)
        cv2.waitKey(0)
        self._as.publish_feedback(self._feedback)

  def unsubscribe(self):
    # use the saved subscriber object to unregister the subscriber
    self.image_sub.unregister()
    
  def goal_callback(self, goal):
    cv2.destroyAllWindows()
  
    self.image_sub = rospy.Subscriber(goal.path, Image, self.callback)
    self._thresh = goal.threshold

    start_time = time.time()
    while True:
      end_time = time.time()
      if self._as.is_preempt_requested():
        rospy.loginfo('The goal has been cancelled/preempted')
        self._as.set_preempted()
        break
      elif end_time-start_time > goal.time_out:
        rospy.loginfo('Success')
        rospy.loginfo(self._feedback.feedback)
        self._as.set_succeeded(self._result)
        break

    self.unsubscribe()
    
if __name__ == '__main__':
  rospy.init_node('object_detect_action_server')
  object_finder()
  rospy.spin()