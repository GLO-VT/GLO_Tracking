# -*- coding: utf-8 -*-
"""
Created on Tue Jun 26 15:57:11 2018

@author: Bailey group PC
"""
import pandas as pd
import minimalmodbus
import time

#ss1 = SS(inst_id=1,com_port='COM6',baudrate=115200)
#timer=[]
#t_start=time.time()
#while time.time()-t_start < 10:
#    ss1.read_data_raw()
#    #time.sleep(0.02)
#    timer.append(time.time())
#timer=pd.Series(np.array(timer)-t_start)
#print('average time',timer.diff().mean())
#ss1.ss.serial.close()

timer=[]
ss=minimalmodbus.Instrument('COM6',1)
ss.serial.baudrate=115200
t_start=time.time()
while time.time()-t_start < 10:
    #ss.read_registers(12,2)
    
    #ss.read_register(12)
    #time.sleep(0.02)
    timer.append(time.time())
timer=pd.Series(np.array(timer)-t_start)
print('average time',timer.diff().mean())
ss.serial.close()