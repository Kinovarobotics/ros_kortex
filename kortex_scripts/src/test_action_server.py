#! /usr/bin/env python
from cmath import rect
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
import json

class qr_decoder(object):
    
  # create messages that are used to publish feedback/result
  _feedback = CustomActionMsgFeedback()
  _result = CustomActionMsgResult()
  _sucess = False
  # decoder varriables
  _bridge = CvBridge()
  _thresh = 100
  _qr_codes_goal = 2
  #image

  def __init__(self):
    # creates the action server
    self._as = actionlib.SimpleActionServer("action_custom_msg_as", CustomActionMsgAction, self.goal_callback, False)
    self._as.start()
    self.ctrl_c = False

  def callback(self,data):
    try:
        cv_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)

    (rows,cols,channels) = cv_image.shape
    image = cv_image
    resized_image = cv2.resize(image, (600, 300)) 
    gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
    img_bw = cv2.threshold(gray, self._thresh, 255, cv2.THRESH_BINARY)[1]
    qr_result = decode(img_bw)

    #cv2.imshow("B&W Image", gray)
    #cv2.imshow("B&W Image /w threshold", img_bw)

    if len(qr_result) > 0:
      x = [None] * len(qr_result)
      y = [None] * len(qr_result)
      w = [None] * len(qr_result)
      h = [None] * len(qr_result)
      for i in range(len(qr_result)):
        (x[i], y[i], w[i], h[i]) = qr_result[i].rect
        resized_image = cv2.rectangle(resized_image, (x[i], y[i]), (x[i] + w[i], y[i] + h[i]), (0, 0, 255), 4)
        text = "{}".format(qr_result[i])
        resized_image = cv2.putText(resized_image, text, (x[i], y[i] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        self._processed_image = resized_image

    code_data = {"number_of_codes_scanned": len(qr_result)}
    for i in range(len(qr_result)):
      key = "qr_code_num"
      numbered_key = key.replace("num", str(len(qr_result)))
      code_data.update({numbered_key :{
        "data": str(qr_result[i].data),
        "rect": { "left": qr_result[i].rect.left,
                  "top" : qr_result[i].rect.top,
                  "width" : qr_result[i].rect.width,
                  "height" : qr_result[i].rect.height
                },
        "polygon": {"point_a": str(qr_result[i].polygon[0]),
                    "point_b": str(qr_result[i].polygon[1]),
                    "point_c": str(qr_result[i].polygon[2]),
                    "point_d": str(qr_result[i].polygon[3])
                   }
      }})

    feedback_string = "found x qr_codes: looking for y qr_codes"
    feedback_string = feedback_string.replace("x",str(len(qr_result)))
    feedback_string = feedback_string.replace("y",str(self._qr_codes_goal))
    self._feedback.feedback = feedback_string
    self._as.publish_feedback(self._feedback)
    
    if len(qr_result) == self._qr_codes_goal:
      cv2.imshow("Camera output", resized_image)
      cv2.waitKey(1000) # not sure why 1000 but just leave it, it works
      self._result.result = json.dumps(code_data)
      self._sucess = True
      self.unsubscribe()

  def unsubscribe(self):
    # use the saved subscriber object to unregister the subscriber
    self.image_sub.unregister()
    
  def goal_callback(self, goal):
    cv2.destroyAllWindows()
    # helper variables
    
    success = True
    
    #   define the different publishers and messages that will be used
    self.image_sub = rospy.Subscriber("/my_gen3/camera/color/image_raw", Image, self.callback)
    
    #if (goal.goal != "LAND"):
    # make the drone takeoff

    while True:
      if success:
        break

    self._result.result = self._feedback.feedback
    rospy.loginfo('Success')
    self._as.set_succeeded(self._result)


if __name__ == '__main__':
  rospy.init_node('action_custom_msg')
  qr_decoder()
  rospy.spin()