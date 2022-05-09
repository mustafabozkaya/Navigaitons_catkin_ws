#! usr/bin/env python3

import rospy
import time 
from std_msgs.msg import Int32 ,String,Float32,Bool,Header
from sensor_msgs.msg import Image
from battery_control_pkg.msg import Batterystatus
from battery_control_pkg.Battery_Control import Battery_Control




class Battery(object):

    def __init__(self,batterycontrol:Battery_Control) -> None:
        super().__init__()
        rospy.init_node('battery_publisher', anonymous=True)
        self.battery_publisher = rospy.Publisher('/battery_status', Batterystatus, queue_size=10) #publisher for battery status
        self.battery_subscriber = rospy.Subscriber('/battery_status', Batterystatus, self.battery_callback) #subscriber for battery status
        self.batterystatu = Batterystatus()
    
    
        self.ctrl_c = False #CREATE FLAG FOR CTRL+C
        self.rate = rospy.Rate(2) #CREATE RATE FOR PUBLISHING
        
        #create battery_control object
        self.battery_control = batterycontrol
    
        rospy.on_shutdown(self.shutdown) #CREATE SHUTDOWN FUNCTION
        #rospy.loginfo("Ctrl c ")

         # publish ones battery status
        self.publish_ones()
      
    def publish_ones(self) -> None:
       self.batterystatu.header = Header() #header for battery status
       self.batterystatu.header.stamp = rospy.Time.now() #time stamp
       self.batterystatu.header.frame_id = "Battery" #frame id
       self.batterystatu.totalvoltage = 57.0 #total voltage
       self.batterystatu.instant_voltage = 0.0 #instantaneous voltage
       self.batterystatu.cells_voltage =[] #cells voltage
       self.batterystatu.cells_volt_difference =[] #cells voltage difference
       self.batterystatu.cells_avarege_volt = 0.0 #cell average voltage
       self.batterystatu.celltempeture1 = 0.0 # cell1 temperature
       self.batterystatu.celltempeture2 = 0.0 # cell2 temperature
       self.batterystatu.Fettempeture1 = 0.0 #fet1 temperature
       self.batterystatu.Pcbtempeture1 = 0.0 #pcb1 temperature
       self.batterystatu.instant_current = 0.0 #instantaneous current
       self.batterystatu.charge_current = 0.0 #charge current
       self.batterystatu.discharge_current = 0.0 #discharge current
       self.batterystatu.battery_capacity = 0.0 #battery capacity
       self.batterystatu.design_capacity = 80 #(AH) #design capacity
       self.batterystatu.Socpercentage = 0.0 # battery state of charge percentage
       self.batterystatu.battery_status = self.batterystatu.POWER_SUPPLY_STATUS_IDLE#battery status (1: charging, 0: discharging, 2: idle, 3: error)
       self.batterystatu.present_status = True #battery present status
       self.batterystatu.cells_temperature = [] #cells temperature
       self.batterystatu.location = "inserted" #battery location
       self.batterystatu.battery_type = self.batterystatu.POWER_SUPPLY_TECHNOLOGY_LION #battery type
       self.batterystatu.serial_number = "123456789" #battery serial number

       while not self.ctrl_c:
            connects=self.battery_publisher.get_num_connections()
            if connects > 0:
                self.battery_publisher.publish(self.batterystatu)
                rospy.loginfo(" first Battery_States  Published")
                break
            else:
                rospy.loginfo("first Battery_publisher not connected")
                time.sleep(1)

    def shutdown(self):
        rospy.loginfo("Stopping Battery_States")
        self.ctrl_c = True

    #create battery_callback function
    def battery_callback(self,data): 
        self.batterystatu.header.stamp = rospy.Time.now()
        
        self.batterystatu.instant_voltage = data.instant_voltage
        self.batterystatu.instant_current = data.instant_current
        self.batterystatu.charge_current = data.charge_current
        self.batterystatu.discharge_current = data.discharge_current
        self.batterystatu.celltempeture1 = data.celltempeture1
        self.batterystatu.celltempeture2 = data.celltempeture2
        self.batterystatu.Fettempeture1 = data.Fettempeture1
        self.batterystatu.Pcbtempeture1 = data.Pcbtempeture1
        
        self.batterystatu.battery_capacity = data.battery_capacity
        self.batterystatu.Socpercentage = data.Socpercentage
        self.batterystatu.battery_status = data.battery_status
    
        self.batterystatu.cells_voltage = data.cells_voltage
        self.batterystatu.cells_avarege_volt = data.cells_avarege_volt
        self.batterystatu.cells_volt_difference = data.cells_volt_difference
        #get each data from batterystatu message
        rospy.loginfo("Battery status Received")
        rospy.loginfo("Battery status: %s", self.batterystatu)
        print("received battery status")

        self.publish_battery_status()
        
        
        
        
    #create publish function for battery status from battery_control module
    def publish_battery_status(self):

        #call battery_instant_voltage function to get instantaneous voltage
        self.batterystatu.instant_voltage = self.battery_control.read_instant_voltage()
        #call battery_instant_current function to get instantaneous current
        self.batterystatu.instant_current = self.battery_control.read_instant_current()
        #call battery_charge_current function to get charge current
        self.batterystatu.charge_current = self.battery_control.read_charge_current()
        #call battery_discharge_current function to get discharge current
        self.batterystatu.discharge_current = self.battery_control.read_discharge_current()
        #call battery_cell_temperature function to get cell1 temperature
        self.batterystatu.celltempeture1 = self.battery_control.read_battery_temperature1()
        #call battery_cell_temperature function to get cell2 temperature
        self.batterystatu.celltempeture2 = self.battery_control.read_battery_temperature2()
        #call battery_fet_temperature function to get fet1 temperature
        self.batterystatu.Fettempeture1 = self.battery_control.read_fet_temperature()
        #call battery_pcb_temperature function to get pcb1 temperature
        self.batterystatu.Pcbtempeture1 = self.battery_control.read_pcb_temperature()
        #call batterystatu function to get battery status (1: charging, 0: discharging, 2: idle, 3: error)
        self.batterystatu.battery_status = self.battery_control.read_battery_status()
        #call battery_soc_percentage function to get battery state of charge percentage
        self.batterystatu.Socpercentage = self.battery_control.read_battery_soc()
        #call battery_cells_voltage function to get battery cells voltage
        self.batterystatu.cells_voltage = self.battery_control.read_cells_voltage()
        #call battery_cells_voltage_difference function to get battery cells voltage difference
        self.batterystatu.cells_volt_difference = self.battery_control.read_cells_voltage_difference()
        #call battery_cell_average_voltage function to get battery cell average voltage
        self.batterystatu.cells_avarege_volt = self.battery_control.read_cells_average_voltage()

        
        while not self.ctrl_c:
            connects=self.battery_publisher.get_num_connections()
            if connects > 0:
                self.battery_publisher.publish(self.batterystatu) #PUBLISH BATTERY STATUS
                rospy.loginfo("Battery_States Published")
                break
            else:
                rospy.loginfo("Battery_publisher not connected")
                
        

    def battery_control(self): #BATTERY CONTROL
        pass

    def battery_monitor(self): #BATTERY MONITOR
        pass

    def battery_status(self): #BATTERY STATUS
        pass



if __name__ == '__main__':
    try:
        port = "/dev/ttyUSB0" # port of the serial connection
        slave_address = 1 # slave address of the battery
        debug = False # debug mode
        battery_control = Battery_Control(port,slave_address,debug) #CREATE OBJECT OF BATTERY CONTROL CLASS


        battery=Battery(battery_control) #CREATE OBJECT OF BATTERY CLASS
        
        battery.rate.sleep() #SLEEP
        rospy.spin() #ROS LOOP
    except rospy.ROSInterruptException as e:
        print(e)

    

   
        






