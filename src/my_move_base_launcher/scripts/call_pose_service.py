#!usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse # Import the service message python classes generated from Empty.srv.


if __name__ == '__main__':
    rospy.init_node('call_pose_service')  # Initialize a ROS node with the name 'get_pose_service'
    rospy.wait_for_service('/get_pose_service') # Wait for the service client /get_pose_service to be running
    try:
        service_connection = rospy.ServiceProxy('/get_pose_service', Empty) # Create the connection to the service
        request = EmptyRequest() # Create an object of type EmptyRequest
        response = service_connection(request) # Send the request to the service.call the service and store the response in a variable called response
        print(f"get pose service response {response}") # Print the response received from the service
        
    except rospy.ServiceException as e: # This exception will be raised if the service call fails
        print("Service call failed: %s"%e) # Print the exception if the service call fails