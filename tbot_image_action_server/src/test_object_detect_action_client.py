#! /bin/python3
import time
import rospy
import actionlib
from tbot_image_action_server.msg import ObjectDetectActionMsgGoal, ObjectDetectActionMsgResult, ObjectDetectActionMsgFeedback, ObjectDetectActionMsgAction

def feedback_callback(feedback):
    print(feedback)

rospy.init_node('test_object_detection_action_client')
client = actionlib.SimpleActionClient('/object_detect_as', ObjectDetectActionMsgAction)
client.wait_for_server()
print("connected to server")

# goal = ObjectDetectActionMsgGoal()
# goal.path = "/camera/depth/image_raw"
# goal.time_out = 1.0
# goal.threshold = 0.025
# print("seending goal")
# client.send_goal(goal, feedback_cb=feedback_callback)
# client.wait_for_result()
# rospy.loginfo("thesh: "+str(goal.threshold))




# state_result = client.get_state()
# rospy.loginfo("state_result: "+str(state_result))

#client.cancel_goal()
counter = 0
while True:
    goal = ObjectDetectActionMsgGoal()
    goal.path = "/camera/depth/image_raw"
    goal.time_out = 3.0
    goal.threshold = 0.01 + 0.01*counter
    print("seending goal")
    client.send_goal(goal, feedback_cb=feedback_callback)
    client.wait_for_result()
    time.sleep(1)
    # state_result = client.get_state()
    rospy.loginfo("thesh: "+str(goal.threshold))
    # rospy.loginfo("state_result: "+str(state_result))
    counter = counter+1
    
    
    
print('[Result] State: %d'%(client.get_state()))