#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String

def gripper_move_publisher():
    pub = rospy.Publisher('/chatter', String, queue_size=10)
    rospy.init_node('gripper_move_publisher', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        hello_str = "1000"
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        gripper_move_publisher()
    except rospy.ROSInterruptException:
        pass