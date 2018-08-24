# -*- coding: utf-8 -*-
"""
Created on Thu Aug 23 20:11:26 2018

@author: addiewan
"""
################################################################################
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import matplotlib as mpl
import os
cwd = os.getcwd()

################################################################################
base_size=20
mpl.rcParams['legend.fontsize'] = base_size
mpl.rcParams['figure.figsize'] = (30,24)
mpl.rcParams['figure.titlesize']=base_size+5
mpl.rcParams['xtick.labelsize']=base_size
mpl.rcParams['ytick.labelsize']=base_size
mpl.rcParams['font.size']=base_size
mpl.rcParams['axes.titlesize']=base_size
mpl.rcParams['axes.labelsize']=base_size
mpl.rcParams['lines.markersize'] = 4           # markersize, in points
mpl.rcParams['legend.markerscale'] = 1     # line width in points
mpl.rcParams['lines.markeredgewidth'] = 0.4 # the line width around the marker symbol
mpl.rcParams['lines.linewidth'] = 4
colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown', 'tab:pink', 'tab:gray', 'tab:olive', 'tab:cyan']
################################################################################
from ptu_newmark import PTU
from ss import SS
import time

ss1 = SS(inst_id=1,com_port='COM6',baudrate=115200)
ptu_x = PTU(com_port='COM11',baudrate=115200)
ptu_y = PTU(com_port='COM9',baudrate=115200)

scan_speed = 80000
scan_increment = 7.5  #degrees for each grid block
scan_x_count = 1
scan_y_count = 1
scan_detect_limit = 4
deg2pos=20000
ss_eshim_x=0.0
ss_eshim_y=0.0

ptu_x.cmd('@01SSPD'+str(scan_speed)+'\r')
ptu_y.cmd('@01SSPD'+str(scan_speed)+'\r')

init_pos_x = 0
init_pos_y = 22.5*deg2pos
step_x = scan_increment*deg2pos
step_y = scan_increment*deg2pos
sleep_x = step_x/scan_speed
sleep_y = step_y/scan_speed
extra_delay=0.5

def scan_check_fov(self):
    ss1.read_data_all()
    ang_x = ss1.ang_x_raw + ss_eshim_x
    ang_y = ss1.ang_y_raw + ss_eshim_y
    if ss1.sun_in_fov:
        steps_x = int(ang_x*deg2pos)
        steps_y = int(ang_y*deg2pos)
        delay_x = steps_x/deg2pos
        delay_y = steps_y/deg2pos
        print('Sun found!')
        ptu_x.cmd('@01X'+str(steps_x)+'\r')
        print('@01X'+str(steps_x)+'\r')
        time.sleep(delay_x)
        ptu_y.cmd('@01X'+str(steps_y)+'\r')
        print('@01X'+str(steps_y)+'\r')
        time.sleep(delay_y)
        ss1.read_data_all()
        print('ang_x=',ss1.ang_x_raw)
        print('ang_y=',ss1.ang_y_raw)
        return False
    else:
        return True

scan_in_progress=True
ptu_x.cmd('@01ABS\r')
ptu_x.cmd('@01X'+str(init_pos_x)+'\r')
time.sleep(init_pos_x/scan_speed + extra_delay)
ptu_x.cmd('@01X'+str(init_pos_y)+'\r')
time.sleep(init_pos_y/scan_speed + extra_delay)
for i in range(8):
    if i % 2 == 1:
        if scan_in_progress:
            ptu_y.cmd(i,'@01X'+str(i*step_y)+'\r')
            print('@01X'+str(i*step_y)+'\r')
            time.sleep(sleep_y + extra_delay)
            scan_in_progress = scan_check_fov()
        if scan_in_progress:
            ptu_x.cmd(i,'@01X'+str(i*step_x)+'\r')
            print('@01X'+str(i*step_x)+'\r')
            time.sleep(sleep_x + extra_delay)
            scan_in_progress = scan_check_fov()
    else:
        if scan_in_progress:
            ptu_y.cmd(i,'@01X'+str(-i*step_y)+'\r')
            print('@01X'+str(-i*step_y)+'\r')
            time.sleep(sleep_y + extra_delay)
            scan_in_progress = scan_check_fov()
        if scan_in_progress:
            ptu_x.cmd('@01X'+str(-i*step_x)+'\r')
            print(i,'@01X'+str(-i*step_x)+'\r')
            time.sleep(sleep_x + extra_delay)
            scan_in_progress = scan_check_fov()
        

        