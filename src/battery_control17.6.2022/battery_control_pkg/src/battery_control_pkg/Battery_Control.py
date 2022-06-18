#!/usr/bin/env python3

import serial
import minimalmodbus
from time import sleep
import numpy as np
import sys
import pandas as pd


class Battery_Control(object):

    def __init__(self,port,slave_adreess,debug_mode):
        self.port = port # port name, for example /dev/ttyUSB0
        self.slave_adreess = slave_adreess # slave address (in decimal)
        self.debug_mode = debug_mode # False for no debug messages, True for debug messages
        self.battery_device = minimalmodbus.Instrument(self.port, self.slave_adreess, self.debug_mode) # create modbus instrument object, connecting to the battery
        self.battery_device.mode = minimalmodbus.MODE_RTU # set the mode to RTU
        self.battery_device.clear_buffers_before_each_transaction = True # clear buffers before each transaction
        self.battery_device.serial.baudrate = 19200 # Baudrate for serial communication (in bps)
        self.battery_device.serial.timeout = 0.5 # Timeout for serial communication in seconds default is 0.5
        self.battery_device.serial.writeTimeout = 0.1 # Write timeout for serial communication in seconds (default is 0.5)
        self.battery_device.serial.parity = serial.PARITY_NONE # No parity for serial communication (not used)
        self.battery_device.serial.stopbits = serial.STOPBITS_ONE # One stop bit for serial communication 
        self.battery_device.serial.bytesize = serial.EIGHTBITS # Eight bits for serial communication
        self.battery_device.serial.xonxoff = False # No Xon/Xoff for serial communication (not used)
        self.battery_device.serial.rtscts = False # No RTS/CTS for serial communication (not used)
        self.battery_device.serial.dsrdtr = False # No DSR/DTR for serial communication (not used)
        self.battery_device.address = self.slave_adreess # set slave address

    

    # read battery voltage (X 0.1V)  
    def read_instant_voltage(self):
        totalvoltage = self.battery_device.read_register(103)  # read voltage (X 100mV) from register
        #print(f"voltage: {totalvoltage/10} (V) \n ") # print voltage (V)
        return totalvoltage/10 # return voltage (V)
        
    
    # read battery current (X 0.01A)
    def read_instant_current(self):
        current = self.battery_device.read_register(104) # return current (X 10mA) ,charge(+) /decharge(-) current from register 
        #print(f"current: {current/100} (A) \n ") # print current (A)
        return current/100 # return current (A)
        

    def read_charge_current(self):
        charge_current = self.read_instant_current()
        if charge_current > 0:
            charge_current = charge_current
        else:
            charge_current = 0.0
        return charge_current
    
    def read_discharge_current(self):
        discharge_current = self.read_instant_current()
        if discharge_current < 0:
            discharge_current = discharge_current
        else:
            discharge_current = 0.0
        return discharge_current
        

    
    # read  1. cell temperature (X 0.1°C) 
    def read_battery_temperature1(self):
        temperature1 = self.battery_device.read_register(147)
        #print(f"temperature1: {temperature1/10} (°C) \n ")
        return temperature1/10 # return temperature1 (°C)
        
    
    # read 2. cell temperature (X 0.1°C)
    def read_battery_temperature2(self):
        temperature2 = self.battery_device.read_register(148)
        #print(f"temperature2: {temperature2/10} (°C) \n ")
        return temperature2/10
        
    
    # read FET temperature (X 0.1°C)
    def read_fet_temperature(self):
        temperature_fet = self.battery_device.read_register(149)
        #print(f"temperature_fet: {temperature_fet/10} (°C) \n ")
        return temperature_fet/10
        

    
    # read battery PCB temperature (X 0.1°C)
    def read_pcb_temperature(self):
        temperature_pcb = self.battery_device.read_register(150)
        #print(f"temperature_pcb: {temperature_pcb/10} (°C) \n ")
        return temperature_pcb/10
      
       # read battery status
    def read_battery_status(self):
        # read battery state (0: Discharging ,1 :Charging ,2: Idle ,3 : Fault) from register
        battery_state = self.battery_device.read_register(114)
        state_dict = {0: "Discharging", 1: "Charging", 2: "Idle", 3: "Fault"}
        #print(f"battery state: {state_dict[battery_state]} \n ") # print battery state
        # return state_dict[battery_state] # return battery state
        return battery_state



    # read Battery  State of Charge (SoC)  (%0-100) from register
    def read_battery_soc(self):
        soc = self.battery_device.read_register(107) # read SoC from register
        #print(f"soc: {soc} % \n ") # print SoC (in %)
        return soc # return SoC (in %)
        

    #read cell voltage (mV) from register
    def read_cell_voltage(self,cell_number):
        cell_voltage = self.battery_device.read_register(cell_number+130)
        #print(f"cell voltage: {cell_voltage} (mV) \n ")
        return cell_voltage

    # read Cell Voltage (X 1mV) from register
    def read_cells_voltage(self):
        cells_voltage = [ {f"cell{i-130}":self.battery_device.read_register(i)} for i in range(131,147)
                                    ] # read cell voltage from register
        
        #print(f"cells voltage: {cells_voltage} (V) \n ") # print cell voltage (V)
        dict_cells_voltage={}
        for dict in cells_voltage:
            dict_cells_voltage.update(dict)
        #print(f"dict cells voltage: {dict_cells_voltage} (V) \n ") # print cell voltage (V)
        #print(f"dict cells vaoltage values: {type(list(dict_cells_voltage.values()))} (V) \n ") # print cell voltage (V)
        # df_cells_voltage=pd.DataFrame(dict_cells_voltage.values(),columns=dict_cells_voltage.keys()) # create dataframe from dictionary
        # print(df_cells_voltage) # print dataframe
        # cell_addrees = cell_number + 130 # cell address between 131 and 146
        # cell_voltage = self.battery_device.read_register(cell_addrees) # read cell voltage (X mV) from register
        # print(f"cell{cell_number} voltage : {cell_voltage} (mV) \n ") # print cell voltage (mV)
        # return cells_voltage # return cell voltage (mV)
        return list(dict_cells_voltage.values())

    
    #read Slave Address of the Battery
    def read_slave_address(self):
        slave_address = self.battery_device.read_register(151)
        #print(f"slave address: {slave_address} \n ") # print slave address
        # return slave_address # return slave address
        return 1

    #read Avarage Cell Voltage (V) from register
    def read_cells_average_voltage(self):
    
        average_cell_voltage = np.array([ self.battery_device.read_register(i) for i in range(133,147)]).mean()
        #print(f"average voltage: {average_cell_voltage/1000} (V) \n ") # # print average voltage (V)
        #print(f"avarage cell voltage: {average_cell_voltage} (mV) \n ") # print average cell voltage (mV)
        # return average_cell_voltage/1000 # return average cell voltage (V),
        return average_cell_voltage

    # calculate cells voltage (mV) difference from avarage cell voltage (mV)
    def read_cells_voltage_difference(self):
        cells_voltage_difference = np.array([ self.battery_device.read_register(i) 
        for i in range(131,147)])- self.read_cells_average_voltage()
        #print(f"cells voltage difference: {cells_voltage_difference} (mV) \n ")
        #print(f"cells voltage difference type : {type(cells_voltage_difference)} \n ")
        return list(cells_voltage_difference)

    #close serial connection
    def close_serial_connection(self):
        self.battery_device.serial.close()
    
    
# if __name__ == '__main__':
#     port = "/dev/ttyUSB0" # port of the serial connection
#     slave_address = 1 # slave address of the battery
#     debug = False # debug mode
#     try :
#         battery= Battery_Control(port,slave_address,debug) # create battery object

#         while True:
#             battery.read_instant_voltage() # read battery voltage (X 0.1V)
#             battery.read_instant_current() # read battery current (X 0.01A)
#             battery.read_battery_status() # read battery status
#             battery.read_battery_temperature1() # read 1. cell temperature (X 0.1°C)
#             battery.read_battery_temperature2() # read 2. cell temperature (X 0.1°C)
#             battery.read_fet_temperature() # read FET temperature (X 0.1°C)
#             battery.read_pcb_temperature() # read battery PCB temperature (X 0.1°C)
#             battery.read_battery_soc() # read Battery  State of Charge (SoC)  (%0-100) from register
#             battery.read_cell_voltage(cell_number=1) # read Cell1 Voltage (X 1mV) from register
#             battery.read_cell_voltage(cell_number=2) # read Cell2 Voltage (X 1mV) from register
#             battery.read_cell_voltage(cell_number=3) # read Cell3 Voltage (X 1mV) from register
#             battery.read_cell_voltage(cell_number=4) # read Cell4 Voltage (X 1mV) from register
#             battery.read_cell_voltage(cell_number=5) # read Cell5 Voltage (X 1mV) from register
#             battery.read_cell_voltage(cell_number=6)  # read Cell6 Voltage (X 1mV) from register
#             battery.read_cell_voltage(cell_number=7)   # read Cell7 Voltage (X 1mV) from register
#             battery.read_cell_voltage(cell_number=8)  # read Cell8 Voltage (X 1mV) from register
#             battery.read_cell_voltage(cell_number=9) # read Cell9 Voltage (X 1mV) from register
#             battery.read_cell_voltage(cell_number=10) # read Cell10 Voltage (X 1mV) from register
#             battery.read_cell_voltage(cell_number=11) # read Cell11 Voltage (X 1mV) from register
#             battery.read_cell_voltage(cell_number=12) # read Cell12 Voltage (X 1mV) from register
#             battery.read_cell_voltage(cell_number=13) # read Cell13 Voltage (X 1mV) from register
#             battery.read_cell_voltage(cell_number=14) # read Cell14 Voltage (X 1mV) from register
#             battery.read_cell_voltage(cell_number=15) # read Cell15 Voltage (X 1mV) from register 
#             battery.read_cell_voltage(cell_number=16) # read Cell16 Voltage (X 1mV) from register

#             battery.read_slave_address() # read Slave Address of the Battery
#             battery.read_cells_voltage_difference() # read avarage cell voltage (V)

#             sleep(1) # sleep for 1 second
#     except Exception as e:
#         print(e)
        
#     finally:
#         # battery.close_serial_connection() # close serial connection
#         print("Battery device closed")
#         sys.exit(1)

