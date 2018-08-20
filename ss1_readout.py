# -*- coding: utf-8 -*-
"""
Created on Sun Jul  8 14:57:47 2018

@author: Bailey group PC
"""

from ss import SS
import time
from datetime import datetime
#ss1 = SS(inst_id=2,com_port='COM4',baudrate=115200)
#ss1 = SS(inst_id=1,com_port='COM6',baudrate=115200)
ss1 = SS(inst_id=4,com_port='COM6',baudrate=115200)
#ss1 = SS(inst_id=1,com_port='COM6',baudrate=115200)
hz=2
rec_time=50000
delay = 1.0/hz            
t0=time.time()
while time.time() - t0 < rec_time:
    d_time=datetime.now()
        
    #Read data
    ss1.read_data_all()
    
    print('x=',ss1.ang_x_raw,'y=',ss1.ang_y_raw)
        
    #Ensure that this loop iterates at desired data rate   
    t_diff = time.time() - t0
    if (delay - t_diff) > 0:
        time.sleep(delay - t_diff)
    else:
        time.sleep(delay)