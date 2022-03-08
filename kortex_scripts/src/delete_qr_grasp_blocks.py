#! /bin/python3

import rospy
from gazebo_msgs.srv import DeleteModel, DeleteModelRequest # Import the service message used by the service /gazebo/delete_model
import sys 

rospy.init_node('remove_model_service_client') # Initialise a ROS node with the name service_client
print("Waiting for Service /gazebo/delete_model")
rospy.wait_for_service('/gazebo/delete_model') # Wait for the service client /gazebo/delete_model to be running
delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel) # Create the connection to the service
kk = DeleteModelRequest() # Create an object of type DeleteModelRequest
kk.model_name = "qr_tomato_block" # Fill the variable model_name of this object with the desired value
print("Deleting model ="+str(kk))
status_message = delete_model_service(kk) # Send through the connection the name of the object to be deleted by the service
print(status_message)
kk.model_name = "qr_pepper_block"
print("Deleting model ="+str(kk))
status_message = delete_model_service(kk)
print(status_message)
