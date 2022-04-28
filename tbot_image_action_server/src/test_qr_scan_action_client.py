#! /bin/python3

import rospy
import actionlib
from kortex_scripts.msg import QRScanActionMsgGoal, QRScanActionMsgResult, QRScanActionMsgFeedback, QRScanActionMsgAction

def feedback_callback(feedback):
    print(feedback)

rospy.init_node('kortex_action_client')
client = actionlib.SimpleActionClient('/qr_scan_as', QRScanActionMsgAction)
client.wait_for_server()
print("connected to server")

goal = QRScanActionMsgGoal()
goal.path = "/camera/color/image_raw"
goal.data = ""
goal.num_codes = 1
goal.time_out = 2.0

client.send_goal(goal, feedback_cb=feedback_callback)
#client.wait_for_result()

state_result = client.get_state()
rospy.loginfo("state_result: "+str(state_result))

#client.cancel_goal()

while state_result < 2:
    
    state_result = client.get_state()
    rospy.loginfo("state_result: "+str(state_result))
    
    
print('[Result] State: %d'%(client.get_state()))