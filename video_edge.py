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




vidcap = cv2.VideoCapture('./Videos/zoovideo3.mp4') 

while True:
    success,frame = vidcap.read()
    if not success:
        break
    
#    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # I think there is a method to read the image directly in gray but this works for now
    # gray_less_noise = signal.convolve2d(gray, anti_noise, boundary='symm', mode='same').astype(np.uint8) # applying the ones filter (from deep learning course) could be used instead if frame
    edges = cv2.Canny(frame, 100, 200) # magic filter, Maybe changing 100, and 200 I dont know what they are yet

    #Grass detection, if want to make it faster, comment this out till new_edges = ..., and use cv2.imshow('frame', edges) Grass only works for zoovideo3
    grassbool1 =  frame[:,:,1] < 220
    grassbool2 = frame[:,:,1] > 120
    grassbool3 = frame[:,:,0] < 200
    grassbool4 = frame[:,:,2] < 200
    grassbool5 = frame[:,:,0] > 60
    grassbool6 = frame[:,:,2] > 90
    grass = (grassbool1*grassbool2*grassbool3*grassbool4*grassbool5*grassbool6*255).astype(np.uint8)

    boolarray = edges > 100
    boolgrass = grass > 250
    shift = 50
    zeros_array = np.zeros((shift,640)).astype(int)
    bool_object = boolarray*1
    bool_grass = (grass > 250)*1
    test1_array = np.concatenate(( zeros_array, bool_grass), axis=0)
    test2_array = np.concatenate((bool_object, zeros_array), axis =0)
    grass_over = (test1_array*test2_array*255).astype(np.uint8)
    over_grass = np.delete(grass_over, range(352,352+shift), 0)
    new_edges = (edges - over_grass).astype(np.uint8)
    

    cv2.imshow('frame', new_edges)
    cv2.waitKey(1)
    
    if cv2.waitKey(1) and 0xFF == ord('q'):
        break
    

