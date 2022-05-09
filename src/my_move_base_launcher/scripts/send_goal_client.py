#! usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseResult,MoveBaseFeedback
from actionlib_msgs.msg import *



# definition of the feedback callback. This will be called when feedback
# is received from the action server
# it just prints a message indicating a new message has been received
def feedback_callback(feedback):
    
    print('[Feedback] Going to Goal Pose...')

# initializes the action client node
rospy.init_node('movebaseGoal_Actionclient')

# create the connection to the action server
client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
# waits until the action server is up and running
client.wait_for_server()

# creates a goal to send to the action server
try:
    while not rospy.is_shutdown():
        goal = MoveBaseGoal()
        for i in range(0,10):
            if i%2==0:
                goal.target_pose.header.frame_id = "map"
            
                goal.target_pose.pose.position.x = 1.16
                goal.target_pose.pose.position.y = -3.76
                goal.target_pose.pose.position.z = 0.0
                goal.target_pose.pose.orientation.x = 0.0
                goal.target_pose.pose.orientation.y = 0.0
                goal.target_pose.pose.orientation.z = 0.75
                goal.target_pose.pose.orientation.w = 0.66
            else:
                goal.target_pose.header.frame_id = "map"
            
                goal.target_pose.pose.position.x = 1.16
                goal.target_pose.pose.position.y = 3.76
                goal.target_pose.pose.position.z = 0.0
                goal.target_pose.pose.orientation.x = 0.0
                goal.target_pose.pose.orientation.y = 0.0
                goal.target_pose.pose.orientation.z = 0.75
                goal.target_pose.pose.orientation.w = 0.66
            print('[Goal] Going to Goal Pose...')
            # sends the goal to the action server, specifying which feedback function
             # to call when feedback received

            # Sends the goal to the action server.
            client.send_goal(goal, feedback_cb=feedback_callback)
            # Waits for the server to finish performing the action.

            wait = client.wait_for_result()
            # If the result doesn't arrive, assume the Server is not available
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
            else:
                # Result of executing the action
                print('[Result] : %d'%(client.get_result()) )




except rospy.ROSInterruptException as e:
    print(e)
    print('[Result] Failed Goal Pose')
    exit(1)

finally:
    print('[Result] Finished')


# Uncomment these lines to test goal preemption:
#time.sleep(3.0)
#client.cancel_goal()  # would cancel the goal 3 seconds after starting

# wait until the result is obtained
# you can do other stuff here instead of waiting
# and check for status from time to time 
# status = client.get_state()
# check the client API link below for more info


    #  goal = MoveBaseGoal()
    #         goal.target_pose.header.frame_id = "map"
    #         goal.target_pose.header.stamp = rospy.Time.now()
    #         goal.target_pose.pose.position.x = 0.0
    #         goal.target_pose.pose.position.y = 0.0
    #         goal.target_pose.pose.position.z = 0.0
    #         goal.target_pose.pose.orientation.x = 0.0
    #         goal.target_pose.pose.orientation.y = 0.0
    #         goal.target_pose.pose.orientation.z = 0.0
    #         goal.target_pose.pose.orientation.w = 1.0
    #         client.send_goal(goal, feedback_cb=feedback_callback)
    #         client.wait_for_result()
    #         print('[Result] State: %d' % (client.get_state()))
    #         print('[Result] Status: %s' % (client.get_goal_status_text()))
    #         print('[Result] Position: [%f, %f, %f]' % (client.get_result().result.pose.pose.position.x, client.get_result().result.pose.pose.position.y, client.get_result().result.pose.pose.position.z))
    #         print('[Result] Orientation: [%f, %f, %f, %f]' % (client.get_result().result.pose.pose.orientation.x, client.get_result().result.pose.pose.orientation.y, client.get_result().result.pose.pose.orientation.z, client.get_result().result.pose.pose.orientation.w))
    #         time.sleep(1)