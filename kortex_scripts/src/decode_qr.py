#!/usr/bin/env python

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode

class camera_1:

  def __init__(self):
    self.image_sub = rospy.Subscriber("/my_gen3/camera/color/image_raw", Image, self.callback)

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
    if len(qr_result) > 0:
      qr_data = qr_result[0].data
      print (qr_result[0])

      ( x, y, w, h) = qr_result[0].rect

      cv2.rectangle(resized_image, (x, y), (x + w, y + h), (0, 0, 255), 4)

      text = "{}".format(qr_data)
      cv2.putText(resized_image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
      cv2.imshow("Camera output", resized_image)

      cv2.waitKey(5)

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

