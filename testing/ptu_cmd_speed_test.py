# -*- coding: utf-8 -*-
"""
Created on Tue Jun 26 15:09:25 2018

@author: Bailey group PC

At 9600 baud, PTU successive commands to ptu cannot be sent
much faster than 8hz. 
"""

t_start=time.time()
print(ptu.read('pp '))
ptu_start=float(ptu.read('pp ').split('is ')[-1])
while time.time()-t_start < 10:
    ptu.cmd('po-1 ')
    time.sleep(0.008)
ptu_stop=float(ptu.read('pp ').split('is ')[-1])
print(' = ',10./(ptu_stop-ptu_start))
print(ptu.read('pp '))