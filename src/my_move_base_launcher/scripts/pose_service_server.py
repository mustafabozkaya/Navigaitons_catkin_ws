#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse # Import the service message python classes generated from Empty.srv.

global robot_pose

def service_callback(request):
    print("Robot Pose:")
    print(robot_pose)
    return EmptyResponse() # the service Response class, in this case EmptyResponse
    
def sub_callback(msg):
    robot_pose = msg.pose.pose

if __name__ == '__main__':
    robot_pose = Pose()
    rospy.init_node('pose_service_server')  # Initialize a ROS node with the name 'get_pose_service'
    my_service = rospy.Service('/get_pose_service', Empty , service_callback) # create the Service called get_pose_service with the defined callback
    
    try: 
        sub_pose = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, sub_callback)
        rospy.spin() # mantain the service open.
         
        
    except rospy.ServiceException as e: # This exception will be raised if the service call fails
        print("Service call failed: %s"%e) # Print the exception if the service call fails