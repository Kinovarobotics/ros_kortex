#! /usr/bin/env python
from unittest import result
import rospy
import time
import actionlib
from kortex_scripts.msg import CustomActionMsgGoal, CustomActionMsgResult, CustomActionMsgFeedback, CustomActionMsgAction
from actionlib.msg import TestFeedback, TestResult, TestAction
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode

class qr_decoder(object):
    
  # create messages that are used to publish feedback/result
  _feedback = CustomActionMsgFeedback()
  _result = CustomActionMsgResult()

  def __init__(self):
    # creates the action server
    self._as = actionlib.SimpleActionServer("action_custom_msg_as", CustomActionMsgAction, self.goal_callback, False)
    self._as.start()
    self.ctrl_c = False

  def callback(self,data):
    bridge = CvBridge()

    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)

    (rows,cols,channels) = cv_image.shape
    image = cv_image
    resized_image = cv2.resize(image, (600, 300)) 

    gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
    thresh = 156
    img_bw = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)[1]

    #print(gray)
    #cv2.imshow("B&W Image", gray)
    #cv2.imshow("B&W Image /w threshold", img_bw)

    qr_result = decode(img_bw)

    #print (qr_result)
    if len(qr_result) > 1:
        qr_data = qr_result[0].data
        print (qr_result[0])

        ( x, y, w, h) = qr_result[0].rect

        cv2.rectangle(resized_image, (x, y), (x + w, y + h), (0, 0, 255), 4)

        text = "{}".format(qr_data)
        cv2.putText(resized_image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.imshow("Camera output", resized_image)

        cv2.waitKey(5)
        self._result.result = str(qr_result[0])
        self.unsubscribe()

  def unsubscribe(self):
    # use the saved subscriber object to unregister the subscriber
    self.image_sub.unregister()
    
  def goal_callback(self, goal):
    # helper variables
    r = rospy.Rate(1)
    success = True
    
    #   define the different publishers and messages that will be used
    self.image_sub = rospy.Subscriber("/my_gen3/camera/color/image_raw", Image, self.callback)
    
    #if (goal.goal != "LAND"):
    # make the drone takeoff
    
    self._result.result = self._feedback.feedback
    rospy.loginfo('Success')
    self._as.set_succeeded(self._result)


if __name__ == '__main__':
  rospy.init_node('action_custom_msg')
  qr_decoder()
  rospy.spin()