# -*- coding: utf-8 -*-
"""
Created on Sat Mar  9 09:35:47 2019

@author: Stavrow
"""
import glob
import numpy as np
import cv2
import pdb
from scipy import signal
import imutils


#YOLO can be used foor deep nets; cnn


kernel = np.ones((20, 20), np.uint8)
vidcap = cv2.VideoCapture('./Videos/zoovideo3.mp4')

while True:
    success,frame = vidcap.read()
    if not success:
        break

    small = cv2.resize(frame, (0,0), fx=0.5, fy=0.5) 
#    frame = small
    compress = 1
    edges = cv2.Canny(frame, 100, 200)

    closing_grass = cv2.morphologyEx(frame, cv2.MORPH_CLOSE, kernel)
    
    grassbool1 =  closing_grass[:,:,1] < 220
    grassbool2 = closing_grass[:,:,1] > 120
    grassbool3 = closing_grass[:,:,0] < 200
    grassbool4 = closing_grass[:,:,2] < 200
    grassbool5 = closing_grass[:,:,0] > 60
    grassbool6 = closing_grass[:,:,2] > 90
    grass = (grassbool1*grassbool2*grassbool3*grassbool4*grassbool5*grassbool6*255).astype(np.uint8)



    boolarray = edges > 100
    boolgrass = grass > 250
    shift = 20
    zeros_array = np.zeros((shift,int(640/compress))).astype(int)
    bool_object = boolarray*1
    bool_grass = (grass > 250)*1
    test1_array = np.concatenate(( zeros_array, bool_grass), axis=0)
    test2_array = np.concatenate((bool_object, zeros_array), axis =0)
    grass_over = (test1_array*test2_array*255).astype(np.uint8)
    over_grass = np.delete(grass_over, range(int(352/compress),352+shift), 0)

    new_edges = (edges - over_grass).astype(np.uint8)
    
#    cnts = cv2.findContours(new_edges.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#    cnts = imutils.grab_contours(cnts)
#    cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:]
#    screenCnt = None
##    print('wat')
#    for c in cnts:
#        peri = cv2.arcLength(c,True)
#        approx = cv2.approxPolyDP(c, 0.015 * peri, True)
##        print('huh')
#        if len(approx) == 4:
#            screenCnt = approx
#            print('test')
#            break


#    cv2.drawContours(frame, [screenCnt], -1, (0, 255, 0), 3)

#    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#    dst = cv2.cornerHarris(gray, 2, 3, 0.04)
#    dst = cv2.dilate(dst, None)
    
#    frame[dst > 0.01*dst.max()] = [0,0,255]
#    cv2.imshow('dst', frame)


    # interesting, but perhaps decreasing size image would have same effect
    closing = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

#    box = 30
#    for i in range(int(255/box)+1):
#        boolgray1 = gray>(255-box*(i+1))
#        boolgray2 = gray<(255-box*i)
#        gray[boolgray1*boolgray2] = 255-box*i if box*i > 0 else 0
#    i_s = []
#    edgebool = boolarray
#    for i in range(len(gray[1,:])):
#        edgelines = edges[:,3*i-2:3*i+2]/255
#        
#        if np.sum(edgelines) >50:
#            i_s.append(2*i-1)
#            print(3*i, np.sum(new_edges))
##    gray[:,i_s] = 255
#    new_gray = gray - clean_edges

    cv2.imshow('frame', closing)
    cv2.waitKey(30)
    
    if cv2.waitKey(1) and 0xFF == ord('q'):
        break
    

