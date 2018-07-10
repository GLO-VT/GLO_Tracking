# -*- coding: utf-8 -*-
"""
Created on Tue Jul 10 13:49:04 2018

@author: Bailey group PC
"""

#2018-07-10 Hang Test data
#plt.figure()
#x=data['data']['run6'].elapsed
#y1=data['data']['run6'].ss2_x_raw
#y2=data['data']['run6'].ang_x_track
#y3=data['data']['run6'].imu_ang_z
#plt.plot(x,y1,label='ss2 x raw')
#plt.plot(x,y2,label='ss2 x track (rolling mean 5)')
#plt.plot(x,y3,label='imu angular rate (deg/sec)')
#plt.legend()

#settle_run6 = data['data']['run6'].elapsed > 50
#plt.figure()
#x=data['data']['run6'].loc[settle_run6,'elapsed']
#y1=data['data']['run6'].loc[settle_run6,'ss2_x_raw']
#y2=data['data']['run6'].loc[settle_run6,'ang_x_track']
#y3=data['data']['run6'].loc[settle_run6,'imu_ang_z']
#plt.plot(x,y1,label='ss2 x raw')
#plt.plot(x,y2,label='ss2 x track (rolling mean 5)')
#plt.plot(x,y3,label='imu angular rate (deg/sec)')
#plt.legend()
#
#plt.figure()
#plt.hist(y1,bins=75)


import numpy as np
import cv2

sun_dia=311 #pixels
deg2pix = 311/0.53
box_len = int((0.53*deg2pix) + (0.2*deg2pix))

cap = cv2.VideoCapture(r"C:\Users\Bailey group PC\Desktop\7_10_18\RUN6\run_20_21.avi")
N=301
video=np.zeros((1024,1024,301))
cnt=0
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read() 
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.rectangle(gray, (500-int(0.1*deg2pix),0), (500-int(0.1*deg2pix)+box_len,box_len), (255, 255, 255), 4)
    video[:,:,cnt]=gray
    # Display the resulting frame
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    cnt+=1

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
#settle_run7 = data['data']['run7'].elapsed > 1
#run='run7'
#plt.figure()
#x=data['data'][run].loc[settle_run7,'elapsed']
#y1=data['data'][run].loc[settle_run7,'ss2_x_raw']
#y2=data['data'][run].loc[settle_run7,'ang_x_track']
#y3=data['data'][run].loc[settle_run7,'imu_ang_z']
#plt.plot(x,y1,label='ss2 x raw')
#plt.plot(x,y2,label='ss2 x track (rolling mean 5)')
#plt.plot(x,y3,label='imu angular rate (deg/sec)')
#plt.legend()
#
#plt.figure()
#plt.hist(y1,bins=75)
