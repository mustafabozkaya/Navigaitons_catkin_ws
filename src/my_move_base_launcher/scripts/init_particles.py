#!/usr/bin/env python
import rospy
from  geometry_msgs.msg import PoseArray
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

if __name__ == '__main__':
    rospy.init_node('init_particles')
    rospy.wait_for_service('/global_localization')
    print('init_particles: waiting for /global_location')
    try:
        global_location = rospy.ServiceProxy('/global_localization', Empty)
        resp = global_location(EmptyRequest())
        print("starting the posisitons of of the particles %s "%resp)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
