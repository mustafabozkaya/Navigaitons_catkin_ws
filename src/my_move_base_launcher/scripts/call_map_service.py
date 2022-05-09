#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid

# This represents a 2-D grid map, in which each cell represents the probability of
# occupancy.

#Header header 

#MetaData for the map
#MapMetaData info

# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
#int8[] data
from nav_msgs.srv import GetMap, GetMapRequest, GetMapResponse, SetMap, SetMapRequest, SetMapResponse
#Getmap.srv is a service message type.
#
#------
#nav_msgs/OccupancyGrid map

import sys 


if __name__ == '__main__':
    rospy.init_node('call_map_service')  # Initialize a ROS node with the name 'call_map_service'
    rospy.wait_for_service('/static_map') # Wait for the service client /static_map to be running
    try: 
        get_static_map = rospy.ServiceProxy('/static_map', GetMap) # Create the connection to the service
        map_request=GetMapRequest() # Create an object of type GetMapRequest
        resp=get_static_map(map_request) # Send the request to the service.call the service and store the response in a variable called resp
        #resp = get_static_map(GetMapRequest()) # Send a service request to the /static_map service with the request message GetMapRequest()
        
         
        #print(resp) # Print the response received from the service
        #print(resp.map.header) # Print the header field of the response received from the service
        print(resp.map.info) # Print the map info received from the service
        #print(resp.map.data) # Print the map data received from the service
    except rospy.ServiceException as e: # This exception will be raised if the service call fails
        print("Service call failed: %s"%e) # Print the exception if the service call fails

    
    