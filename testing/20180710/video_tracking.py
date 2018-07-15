# -*- coding: utf-8 -*-
"""
Created on Tue Jul 10 16:21:07 2018

@author: addiewan
"""
from plot_ss_data_new import grab_data
import os
import matplotlib.pyplot as plt
import numpy as np
import cv2
cwd = os.getcwd()

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