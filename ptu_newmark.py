# -*- coding: utf-8 -*-
"""
Created on Thu May 17 14:12:47 2018

@author: Bailey group PC
"""

import serial
import time
from datetime import datetime
import numpy as np
try:
    import ephem
except:
    print('Unable to import ephem, cannot command PTU to point at sun or moon')
try:
    from vnpy import EzAsyncData
except:
    print('Unable to import vnpy (VN100 IMU library), cannot use IMU for sun/moon pointing')

#import pandas as pd

class PTU:
    '''
    Class to connect to FLIR Pan Tilt Unit
    
    Inputs:
        com_port: serial com port for USB to 485-converter
        baudrate: (int) default baudrate = 19200, fastest baudrate = 115200
        cmd_delay: (int) delay (in seconds) to wait between ptu commands
    '''
    def __init__(self,com_port='COM9',baudrate=9600,cmd_delay=0.025):
        self.com_port = com_port
        self.baudrate = baudrate
        self.cmd_delay = cmd_delay

        #Establish communication with PTU
        self.ptu = serial.Serial(port=self.com_port,baudrate=self.baudrate)
        if self.ptu.isOpen():
            self.cmd('i ')    #Send this command to PTU to make it respond to commands
            print('Connected to PTU D300 Ebay (or not)')
     
    def read(self,command,delay=0.010):
        '''
        Send ptu a command, and return the ptu response
        '''
        try:
            self.ptu.write(command.encode())
            time.sleep(delay)
            bytesToRead = self.ptu.inWaiting()
            time.sleep(delay)
            ptu_out = self.ptu.read(bytesToRead)
            #print(ptu_out)
            ptu_out = ptu_out.decode().split('\r')[0]
            return ptu_out
        except:
            print('Could not read command from PTU')
            return
        
    def cmd(self,command,delay=0.005):
        '''
        Send command to PTU
        '''
        self.ptu.write(command.encode())
        time.sleep(delay)
        return
        
     
                  
if __name__ == '__main__':
    
    #Pan the PTU at max speed in one direction, stop, and pan in reverse direction
    #Create a ptu object, define com_port and baudrate
    ptu_x = PTU(com_port='COM9',baudrate=9600)
    ptu_x.read('@01PX\r')
##    ptu_x.cmd('@01J+\r')  #Set to positive velocity mode
##    time.sleep(0.1)
##    ptu_x.cmd('@01SSPD80000\r')  #move at 50,000 pos/sec
##    time.sleep(10)
##    ptu_x.cmd('@01STOP\r')
##    time.sleep(0.1)
##    ptu_x.cmd('@01J-\r')  #Set to positive velocity mode
##    time.sleep(0.1)
##    ptu_x.cmd('@01SSPD80000\r')  #move at 50,000 pos/sec
##    time.sleep(10)
#    ptu_x.cmd('@01STOP\r')
#    time.sleep(0.1)
#    ptu_x.cmd('@01J+\r')
#    x=80000*np.sin(np.linspace(0,2*np.pi,100)) 
#    for i in range(len(x)):
#        time.sleep(0.05)
#        ptu_x.cmd('@01SSPD'+str(x[i])+'\r')  #move at 50,000 pos/sec
#    time.sleep(0.05)
#    ptu_x.cmd('@01STOP\r')
#    time.sleep(0.05)
#    ptu_x.cmd('@01J-\r')
#    time.sleep(0.05)
#    for i in range(len(x)):
#        time.sleep(0.05)
#        ptu_x.cmd('@01SSPD'+str(x[i])+'\r')  #move at 50,000 pos/sec