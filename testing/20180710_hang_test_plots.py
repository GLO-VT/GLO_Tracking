# -*- coding: utf-8 -*-
"""
Created on Tue Jul 10 13:49:04 2018

@author: Bailey group PC
"""
from plot_ss_data_new import grab_data
import os
import matplotlib.pyplot as plt
import cv2
cwd = os.getcwd()

ptu_pos2deg=23./3600
header_len=2
data_loc =  cwd+'/20180710'
data = grab_data(data_loc,header_len=header_len)

#2018-07-10 Hang Test data
plt.figure()
x=data['data']['run6'].elapsed
y1=data['data']['run6'].ss2_x_raw
y2=data['data']['run6'].ang_x_track
y3=data['data']['run6'].imu_ang_z
plt.plot(x,y1,label='ss2 x raw')
plt.plot(x,y2,label='ss2 x track (rolling mean 5)')
plt.plot(x,y3,label='imu angular rate (deg/sec)')
plt.legend()

settle_run6 = data['data']['run6'].elapsed > 50
plt.figure()
x=data['data']['run6'].loc[settle_run6,'elapsed']
y1=data['data']['run6'].loc[settle_run6,'ss2_x_raw']
y2=data['data']['run6'].loc[settle_run6,'ang_x_track']
y3=data['data']['run6'].loc[settle_run6,'imu_ang_z']
y4=data['data']['run6'].loc[settle_run6,'imu_filt_x']
y5=data['data']['run6'].loc[settle_run6,'ptu_cmd_x']
plt.plot(x,y1,label='ss2 x raw')
plt.plot(x,y2,label='ss2 x track (rolling mean 5)')
plt.plot(x,-y3,label='imu angular rate (deg/sec)')
plt.plot(x,-y4*180./np.pi,label='filtered imu angular rate (deg/sec)')
plt.plot(x,y5*ptu_pos2deg,label='ptu_cmd (deg/sec)')
plt.legend()

plt.figure()
plt.hist(y1,bins=75)


#import numpy as np
#import cv2
#
#sun_dia=311 #pixels
#deg2pix = 311/0.53
#box_len = int((0.53*deg2pix) + (0.2*deg2pix))
#
#cap = cv2.VideoCapture(r"C:/Users/addiewan/Desktop/RUN6/run_20_21.avi")
#N=301
#video=np.zeros((1024,1024,301))
#cnt=0
#while(True):
#    # Capture frame-by-frame
#    ret, frame = cap.read() 
#    # Our operations on the frame come here
#    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#    cv2.rectangle(gray, (500-int(0.1*deg2pix),0), (500-int(0.1*deg2pix)+box_len,box_len), (255, 255, 255), 4)
#    video[:,:,cnt]=gray
#    # Display the resulting frame
#    cv2.imshow('frame',gray)
#    if cv2.waitKey(1) & 0xFF == ord('q'):
#        break
#    cnt+=1
#
## When everything done, release the capture
#cap.release()
#cv2.destroyAllWindows()
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

ptu_pos2deg=23./3600
header_len=2
data_loc =  cwd+'/20180710'
data = grab_data(data_loc,header_len=header_len)



def track(frame, mask):
    """
    Track the position of the laser pointer.

    Code taken from
    http://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
    """
    x, y = None, None
    radius = None

    countours = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                 cv2.CHAIN_APPROX_SIMPLE)[-2]

    # only proceed if at least one contour was found
    if len(countours) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(countours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        moments = cv2.moments(c)
        if moments["m00"] > 0:
            center = int(moments["m10"] / moments["m00"]), \
                     int(moments["m01"] / moments["m00"])
        else:
            center = int(x), int(y)
        #print(x,y,radius)
        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            cv2.circle(frame, (int(x), int(y)), int(radius),
                       (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            # then update the ponter trail
            #if self.previous_position:
             #   print(center)
             #   cv2.line(self.trail, self.previous_position, center,
             #            (255, 255, 255), 2)

    #cv2.add(self.trail, frame, frame)
    return x, y, radius
    
#
vid_loc=r"C:\Users\addiewan\Desktop\RUN6\run_20_21.avi"
sun_dia=311 #pixels
deg2pix = 311/0.53
pix2deg = 0.53/311
box_len = int((0.53*deg2pix) + (0.2*deg2pix))
gauss_kern=7
cap = cv2.VideoCapture(vid_loc)
N=300
#video=np.zeros((1024,1024,301))
cnt=0
x=[]
y=[]
r=[]
for i in range(N):
    # Capture frame-by-frame
    ret, frame = cap.read() 
    # Our operations on the frame come here
    #gray = frame[:,:,0]
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    dum,thresh = cv2.threshold(gray,75,200,cv2.THRESH_BINARY)
    gauss = cv2.GaussianBlur(thresh,(gauss_kern,gauss_kern),0)
    x_cen, y_cen, radius=track(gray, gauss)
    x.append(x_cen)
    y.append(y_cen)
    r.append(radius)
    #cv2.rectangle(gray, (500-int(0.1*deg2pix),0), (500-int(0.1*deg2pix)+box_len,box_len), (255, 255, 255), 4)
    
    #video[:,:,cnt]=gray
    # Display the resulting frame
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    cnt+=1
    
x_deg = (np.array(x )- x[0])*pix2deg 
y_deg = (np.array(y) - y[0])*pix2deg 
r_deg = np.array(r)*pix2deg 

mask_run6=(data['data']['run6'].elapsed >3) &(data['data']['run6'].elapsed <6)
y1=data['data']['run6'].loc[mask_run6,'ss2_x_raw']
plt.figure()
plt.plot(np.arange(len(y1)),y1)
plt.plot(np.arange(len(x_deg)),x_deg)