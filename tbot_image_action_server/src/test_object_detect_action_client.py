#! /bin/python3

import rospy
import actionlib
from tbot_image_action_server.msg import ObjectDetectActionMsgGoal, ObjectDetectActionMsgResult, ObjectDetectActionMsgFeedback, ObjectDetectActionMsgAction

def feedback_callback(feedback):
    print(feedback)

rospy.init_node('test_object_detection_action_client')
client = actionlib.SimpleActionClient('/object_detect_as', ObjectDetectActionMsgAction)
client.wait_for_server()
print("connected to server")

goal = ObjectDetectActionMsgGoal()
goal.path = "/my_gen3/camera/depth/image_raw"
goal.time_out = 60.0
goal.threshold = 0.09

client.send_goal(goal, feedback_cb=feedback_callback)
#client.wait_for_result()

state_result = client.get_state()
rospy.loginfo("state_result: "+str(state_result))

#client.cancel_goal()

while state_result < 2:
    
    state_result = client.get_state()
    rospy.loginfo("state_result: "+str(state_result))
    
    
print('[Result] State: %d'%(client.get_state()))