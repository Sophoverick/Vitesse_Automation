#!/usr/bin/env python
'''
Important Team Vitesse Automation Member do read this:
dronefunctions and AIfunctions.py should be in the same directory,
make sure they are open to understand a clue, AIfunctions contains all bbox calculation
and drone triangulation function, dronefunctions contain simplified pymavlink
'''
from __future__ import print_function

import roslib
roslib.load_manifest('vit')
from pymavlink import mavutil
import numpy as np
import rospy
import sys
import cv2
import torch
import torchvision
from time import sleep
from math import*
from dronefunctions import*
from AIfunctions import*
from vit.msg import target
from PIL import Image as Img
from ultralytics import YOLO
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#initializing parameters etc., some are in functs.py
yawtol=0.1 #minimum acceptable yaw
shootingrange=3 #range to stay in
mainimage = None #declaring object for later use in callback function, always updated
model=YOLO('saved_model.pt')
print("loaded model")
the_connection = mavutil.mavlink_connection('udpin:localhost:14550')
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
(the_connection.target_system, the_connection.target_component))
#I had to do it here so that the main image could be forwarded
def dtriangulate(connection, model, orpitch):
  oralt=raltitude(connection)
  #go down and measure pitch from there
  print("Going low")
  msg=relup2(connection,-1,0.25,0.25)
  yaw, lowpitch, lowdetected=process_imgsim(model, mainimage)
  lowalt=raltitude(connection)
  #just so yk, lowdetected will be 1 for detected updetected will be 2
  print("Going high")
  msg=relup2(connection,2,0.25,0.25)
  yaw, uppitch, updetected=process_imgsim(model,mainimage)
  updetected*=2
  upalt=raltitude(connection)
  detected=lowdetected+updetected
  if detected == 0:
    print("Lost sight of drone")
    target_alt, forward=[0,0]
  elif detected == 1:
    print("Drone pitched just from below")
    target_alt, forward=tcalculate(lowalt,oralt, lowpitch, orpitch)
  elif detected == 2:
    print("Drone pitched just from above")
    target_alt, forward=tcalculate(oralt,upalt, orpitch, uppitch)
  elif detected == 3:
    print("Drone pitched from above and below")
    target_alt, forward=tcalculate(lowalt,upalt, lowpitch, uppitch)
  #return the altitude and how forward the target is
  print("The target drone has an altitude %.2f m and is %.2f m forward" %
         (target_alt, forward))
  return target_alt, forward, detected

#defines ROS aspect with subscriber
class image_inferrer:
  def __init__(self):
    self.bridge = CvBridge()
    image_sub = rospy.Subscriber("/webcam/image_raw",Image,self.callback,queue_size=1)
    #publish angle in radians and image 
    #self.image_pub = rospy.Publisher("bb_ranges",target)
  def callback(self,Image):
    #saves a frame from the topic
    try:
      global mainimage
      cv_image = self.bridge.imgmsg_to_cv2(Image, "bgr8")
      mainimage=cv_image #save image in the global variable
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_inferrer()
  rospy.init_node('image_inferrer', anonymous=True)
  rate = rospy.Rate(1)
  while (True):
    try:
      print("Since callback has been done, image processing gonna start")
      yaw, pitch, detected=process_imgsim(model,mainimage)
      if detected==1:
        if yaw > yawtol:
          msg=relyaw2(the_connection,yaw,1,0.1)
          print("Yawing done, repeating")
        else:
          print("Triangulation begun")
          target_alt, forward, detected=dtriangulate(the_connection, model, pitch)
          #go to slightly above desired altitude so shooting becomes easier
          if detected!=0:
            msg=gotoalt2(the_connection,target_alt+0.1,0.25, 0.05)
            print("inline with the drone")
            yaw, pitch, detected=process_imgsim(model,mainimage)
            if detected==1:
              #yaw again with more accuracy
              relyaw2(the_connection,yaw,1,0.1)
              #save the yaw absolutely, if it misyaws when going forward
              msg=rattitude(the_connection)
              target_yaw=msg.yaw
              advance=forward-shootingrange
              print("Gonna advance %.2f m", advance)
              msg=relforward2(the_connection, advance, 1, 0.1)
              print("Reached the location, now orienting correctly")
              msg=gotoalt2(the_connection,target_alt+0.1,0.25, 0.05)
              msg=gotoyaw2(the_connection,target_yaw,0.5,0.1)
              print("Reached shooting range, FIRING!!")
            else:
              print("Lost sight of drone, passing")
              pass
          else:
            pass
      else:
        print("No bounding boxes detected")
        pass
      rate.sleep()
      print("Repeating")
    except KeyboardInterrupt:
      print("Shutting down")
      break
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)