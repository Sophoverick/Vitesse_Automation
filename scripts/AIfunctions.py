#!/usr/bin/env python
'''
Important Team Vitesse Automation Member do read this:
These contain variety of functions for both simulation and physical implementation, in both
cases, functions are named so they can be identified while being reviewed for their purpose.
'''
import numpy as np
import cv2
import torch
import torchvision
from math import*
from dronefunctions import*
from PIL import Image as Img
from ultralytics import YOLO

width=640 #width of image 
height=480 #height of image
hfov=1.2 #Horizontal FOV
vfov=0.9 #Vertical FOV
center_img = (width/2, height/2)
#for vertical triangulate function l for lower position h for higher position
def tcalculate(lalt,halt,lpitch,hpitch):
  if tan(hpitch) <= 0.05:
    target_alt=halt
  else:
    x=tan(lpitch)/tan(hpitch)
    #calculation derivation available in pdf
    if abs(x-1)<=0.05: #less then 3 degrees of difference leads to this 
      #altitude of target relative to ground (not drone):
      print("The ratio of tangents of pitches is %.2f"%x)
      target_alt=(x*halt-lalt)/(x-1)
      print("The target was %.2f above the highest height", target_alt-halt)
    else:
      target_alt=(halt+lalt)/2
  #relative forward distance from drone to target
  forward=abs(target_alt-lalt)/abs(tan(lpitch))
  return target_alt, forward

#main thing that calculates distance to yaw and pitch
def process_imgsim(model, frame):
    resultsls = model.predict(frame, conf=0.7)
    result=resultsls[0]
    boxes = result.boxes
    boxarray=np.array(boxes.xyxy)
    bboxes=len(boxarray)
    if bboxes>=1:
      data =result.boxes.xywhn
      point = np.array([])
      area = np.array([])
      i=0
      for x,y,w,h in data:
        point = np.append(point,[x,y,w,h])
        point = np.reshape(point,(int(len(point)/4),4))
        area = np.append(area,w*h)
      closest = np.argmax(area)
      box=boxes[closest]
      print("Closest bounding box selected")
      xyxy = box.xyxy[0]
      x_min, y_min, x_max, y_max = xyxy
      center_box = ((x_min + x_max) / 2, (y_min + y_max) / 2)
      print("target drone center at %.2f,%.2f of pixel" %(center_box[0],center_box[1]))
      delta_x = (center_box[0] - center_img[0])/width #delta as percentage
      delta_y = (center_box[1] - center_img[1])/height #delta as percentage
      yaw=delta_x*hfov #angle in radians
      pitch=-delta_y*vfov #angle in radians, minus to adjust with opencvs frame definition
      print("yaw and pitch (%.2f, %.2f) forwarded"% (yaw, pitch))
      #cv2.imshow("Image window", result.plot())
      #cv2.waitKey(10000)
      detected=1 # It was detected
    else:
      #cv2.imshow("Image window", result.plot())
      #cv2.waitKey(10000)
      yaw=0
      pitch=0
      detected=0
    return yaw, pitch, detected