# create same_node_pub_subs.py
# create Ros publisher and subscriber node messages type Baterystatus from battery_control_pkg/msg/Batterystatus.msg

import rospy
from battery_control_pkg.msg import Batterystatus
from battery_control_pkg.src.battery_control_pkg.Battery_Control import Batterycontrol
from std_msgs.msg import String,Header,Float32,Bool,unint8
import time


class Battery(object):
    
    def __init__(self) -> None:
        super().__init__()
        rospy.init_node('Battery_States', anonymous=True)
        self.battery_publisher = rospy.Publisher('/battery_status', Batterystatus, queue_size=10) #publisher for battery status
        self.battery_subscriber = rospy.Subscriber('/battery_status', Batterystatus, self.battery_callback) #subscriber for battery status

        self.battery_status = Batterystatus() #create battery_status instance

       
        self.ctrl_c = False
        self.rate = rospy.Rate(1)
        self.sleeptime=time.sleep(1.0) # for 1 seconds
        rospy.on_shutdown(self.shutdownhook) #shutdown hook function 
    

    def shutdownhook(self):
        self.ctrl_c = True  # triggers shutdown

    
   
  

  
        

        

