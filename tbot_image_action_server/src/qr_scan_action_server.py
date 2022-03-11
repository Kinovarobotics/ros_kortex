#! /bin/python3
from cmath import rect
from enum import unique
from tracemalloc import start
from unittest import result
import rospy
import time
import actionlib
from kortex_scripts.msg import QRScanActionMsgGoal, QRScanActionMsgResult, QRScanActionMsgFeedback, QRScanActionMsgAction
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
  _feedback = QRScanActionMsgFeedback()
  _result = QRScanActionMsgResult()
  _success = False
  # decoder varriables
  _bridge = CvBridge()
  _thresh = 100
  _qr_codes_goal = 1
  _target = ""

  def __init__(self):
    # creates the action server
    self._as = actionlib.SimpleActionServer("qr_scan_as", QRScanActionMsgAction, self.goal_callback, False)
    self._as.start()
    self.ctrl_c = False

  def callback(self,data):
    unique_codes = {"num": 0}

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

    for i in range(len(qr_result)):
      if str(qr_result[i].data)[2:-1] not in unique_codes:
        unique_codes.update({str(qr_result[i].data)[2:-1] :{
          "data": str(qr_result[i].data)[2:-1],
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
        unique_codes["num"] = (unique_codes.get("num") + 1)

    key_list = list(unique_codes.keys())
    key_list.pop(0)
    if not self._target: 
      self._feedback.feedback = "found: " + str(unique_codes["num"]) + ", want: " + str(self._qr_codes_goal) + " unique qr codes, data: " + str(key_list)
    else:
      self._feedback.feedback  = "found: " + str(key_list) + ", target: " + self._target
    self._as.publish_feedback(self._feedback)
    
    if (unique_codes["num"] == self._qr_codes_goal and not self._target) or self._target in unique_codes:
      cv2.imshow("Camera output", resized_image)
      cv2.waitKey(1000) # not sure why 1000 but just leave it, it works
      print(unique_codes)
      self._result.result = json.dumps(unique_codes)
      self._success = True
      self.unsubscribe()

  def unsubscribe(self):
    # use the saved subscriber object to unregister the subscriber
    self.image_sub.unregister()
    
  def goal_callback(self, goal):
    cv2.destroyAllWindows()
  
    self.image_sub = rospy.Subscriber(goal.path, Image, self.callback)
    self._qr_codes_goal = goal.num_codes
    self._target = goal.data
    
    start_time = time.time()
    
    while True:
      end_time = time.time()
      if self._success:
        rospy.loginfo('Success')
        self._as.set_succeeded(self._result)
        break
      elif self._as.is_preempt_requested():
        rospy.loginfo('The goal has been cancelled/preempted')
        self._as.set_preempted()
        self.unsubscribe()
        break
      elif end_time-start_time > goal.time_out:
        rospy.loginfo('Time out reached before reaching goal')
        self._as.set_aborted()
        self.unsubscribe()
        break
    
if __name__ == '__main__':
  rospy.init_node('qr_scan_action_server')
  qr_decoder()
  rospy.spin()