#! /usr/bin/env python

import rospy 
import numpy as np                                        
from std_msgs.msg import Int32 
from sensor_msgs.msg import LaserScan
import time as tm
from std_msgs.msg import Int32 ,String
from geometry_msgs.msg import Twist


cmd=Twist()
cmd.angular.x=1.0
cmd.angular.y=0.0
cmd.angular.z=0.0
cmd.linear.x=0.0
cmd.linear.y=0.0
cmd.linear.z=0.0

def publish_cmd(cmd):

    rospy.init_node('cmd_publisher')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(2)

    while not rospy.is_shutdown(): 
    pub.publish(cmd)
    rate.sleep()

def callback(msg):                                    # Define a function called 'callback' that receives a parameter 
    list_arr=list(msg.ranges)                                       # named 'msg'
    narr=np.array(list_arr)
    wall_dist=narr[350:370].mean()
    print (wall_dist)                                 # Print the value 'data' inside the 'msg' parameter
    # print (list_arr[205:440])                                  # Print the value 'data' inside the 'msg' parameter
    # print (list_arr[350:370])  

    
        if wall_dist <=1.0:


rospy.init_node('scan_subcriber')
sub = rospy.Subscriber('/kobuki/laser/scan', LaserScan, callback)   # Create a Subscriber object that will listen to the /counter
                                                      # topic and will cal the 'callback' function each time it reads
                                                      # something from the topic
rospy.spin()                                          # Create a loop that will keep the program in execution





