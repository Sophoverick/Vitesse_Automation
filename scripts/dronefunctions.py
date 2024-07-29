#!/usr/bin/env python3
'''
Important, do read this:
This function could see some errors if somethign is wrong with the connection,
the basic drill of GPS locking etc. prior to flight is known and implemented.
However due to some problems some messages might not be able to reach
the jetson nano via MAVLink, for that case I have made 2 copies of each movement
function. The r for relative in most functions refers relative to
drone NOT to the ground. Gotoalt is function which takes relative to ground while most others are
relative to the drone for ease of use.
Functions like relup2, relyaw2, relforward2 involve the absence of ACKs and instead use
location functions and feedback to wait till the destination has been reached.
First try using the first ones, if COMMAND_ACK and MISSION_ACK work then their is no need to look
at the functions suffixed with 2
'''

from pymavlink import mavutil
import numpy as np
import math
from time import sleep

#This function prints relative location, location relative to start
def rlocation(connection):
    #intake NED position as [x y z] where units are meters
    msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    #print("The position is (%.4f,%.4f,%.4f)"% (msg.x,msg.y,msg.z)
    #      ,end="")
    return msg

#This function prints relative altitude, altitude relative to ground
def raltitude(connection):
    #intake NED position as [z] where units are meters
    msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    #print("The altitude is (%.2f)"% (-msg.z)
    #   ,end="")
    return (-msg.z)

#This function prints roll,pitch and yaw
def rattitude(connection):
    #intake Attitude in radians
    msg = connection.recv_match(type='ATTITUDE', blocking=True)
    #print(" with attitudes of (%.4f,%.4f,%.4f) "% (msg.roll,msg.pitch,msg.yaw))
    return msg

#This function increments position, so we don't have to enter manually
def relmove_drone(connection,darr,speed,angspeed):
    #obtaining local coordinates
    msgl=rlocation(connection)
    msga=rattitude(connection)
    arr=np.array([msgl.x,msgl.y,msgl.z,msga.yaw])
    #transforming difference array darr
    connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, connection.target_system,
                        connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                            int(0b100111111000),arr[0]+darr[0],arr[1]+darr[1],arr[2]+darr[2],speed,speed,speed,0,0,0,arr[3]+darr[3],angspeed))
    connection.recv_match(type='COMMAND_ACK', blocking=True)
    print("Undergoing displacement")
    connection.recv_match(type='MISSION_ACK', blocking=True)
    #Calming down for inertial measurements
    sleep(2)
    msg = rlocation(connection)
    return msg

#This function increments yaw manually, yaw to the right though in radians
def relyaw(connection,yaw,angspeed):
    msgl=rlocation(connection)
    msga=rattitude(connection)
    arr=np.array([msgl.x,msgl.y,msgl.z,msga.yaw])
    darr=np.array([0,0,0,yaw])
    oryaw=arr[3]
    newyaw=arr[3]+yaw
    connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, connection.target_system,
                        connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                            int(0b100111111000),arr[0],arr[1],arr[2],1,1,1,0,0,0,newyaw,angspeed))
    connection.recv_match(type='COMMAND_ACK', blocking=True)
    print("Undergoing displacement")
    connection.recv_match(type='MISSION_ACK', blocking=True)
    msg = connection.recv_match(type='ATTITUDE', blocking=True)
    yaw=msg.yaw
    print("The current yaw has become (%.2f)"% (yaw))
    #Calming down for inertial measurements
    sleep(2)
    return msg

#incase relyaw doesn't work due to command ack, tol is position tolerated 
def relyaw2(connection,yaw,angspeed, tol):
    msgl=rlocation(connection)
    msga=rattitude(connection)
    arr=np.array([msgl.x,msgl.y,msgl.z,msga.yaw])
    darr=np.array([0,0,0,yaw])
    oryaw=arr[3]
    newyaw=arr[3]+yaw
    msg=connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, connection.target_system,
                        connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                            int(0b100111111000),arr[0],arr[1],arr[2],1,1,1,0,0,0,newyaw,angspeed))
    msg=connection.recv_match(type='ATTITUDE', blocking=True)
    yaw=msg.yaw
    while(abs(newyaw-yaw) > tol):
        print("not there")
        msg=connection.recv_match(type='ATTITUDE', blocking=True)
        yaw=msg.yaw
        pass
    #Calming down for inertial measurements
    sleep(4)
    print("The current yaw has become (%.2f)"% (yaw))
    return msg

#move up in meters
def relup(connection,up,speed):
    #obtaining local coordinates
    msgl=rlocation(connection)
    msga=rattitude(connection)
    arr=np.array([msgl.x,msgl.y,msgl.z,msga.yaw])
    darr=np.array([0,0,-up,0])
    #transforming difference array darr
    connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, connection.target_system,
                        connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                            int(0b100111111000),arr[0],arr[1],arr[2]+darr[2],speed,speed,speed,0,0,0,arr[3],0.25))
    connection.recv_match(type='COMMAND_ACK', blocking=True)
    print("Undergoing displacement")
    connection.recv_match(type='MISSION_ACK', blocking=True)
    #Calming down for inertial measurements
    sleep(2)
    msg = raltitude(connection)
    return msg

#just in case the previous functions don't work
def relup2(connection, up, speed,tol):
    #obtaining local coordinates
    msgl=rlocation(connection)
    msga=rattitude(connection)
    arr=np.array([msgl.x,msgl.y,msgl.z,msga.yaw])
    darr=np.array([0,0,-up,0])
    newheight=arr[2]+darr[2]
    #transforming difference array darr
    connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, connection.target_system,
                        connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                            int(0b100111111000),arr[0],arr[1],arr[2]+darr[2],speed,speed,speed,0,0,0,arr[3],0.25))
    msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    height=msg.z
    while(abs(newheight-height) > tol):
        print("not there")
        msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        height=msg.z
        pass
    #Calming down for inertial measurements
    sleep(4)
    print("The current height has become (%.2f)"% (-msg.z))
    return msg

#move forward in direction being faced with FRD
def relforward(connection,dist,speed):
    #obtaining local coordinates
    msgl=rlocation(connection)
    msga=rattitude(connection)
    arr=np.array([msgl.x,msgl.y,msgl.z,msga.yaw])
    yaw=msga.yaw
    dx=math.cos(yaw)*dist
    dy=math.sin(yaw)*dist
    darr=np.array([dx,dy,0,0])
    connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, connection.target_system,
                    connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                        int(0b100111111000),arr[0]+darr[0],arr[1]+darr[1],arr[2],speed,speed,speed,0,0,0,arr[3],0.25))
    connection.recv_match(type='COMMAND_ACK', blocking=True)
    print("Undergoing displacement")
    connection.recv_match(type='MISSION_ACK', blocking=True)
    #Calming down for inertial measurements
    sleep(2)
    msg = rlocation(connection)
    return msg

#incase relforward doesn't work, tol is total error in position tolerated
def relforward2(connection,dist,speed, tol):
#obtaining local coordinates
    msgl=rlocation(connection)
    msga=rattitude(connection)
    arr=np.array([msgl.x,msgl.y,msgl.z,msga.yaw])
    yaw=msga.yaw
    dx=math.cos(yaw)*dist
    dy=math.sin(yaw)*dist
    darr=np.array([dx,dy,0,0])
    newarr=arr+darr
    connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, connection.target_system,
                    connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                        int(0b100111111000),newarr[0],newarr[1],newarr[2],speed,speed,speed,0,0,0,newarr[3],0.25))
    msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    arr=np.array([msg.x,msg.y,msg.z])
    x=abs(newarr[0]-arr[0])+abs(newarr[1]-arr[1])+abs(newarr[2]-arr[2])
    while(x > tol):
        print("not there")
        msg=connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        arr=np.array([msg.x,msg.y,msg.z])
        x=abs(newarr[0]-arr[0])+abs(newarr[1]-arr[1])+abs(newarr[2]-arr[2])
        pass
    print("The position has become (%.4f,%.4f,%.4f)"% (msg.x,msg.y,msg.z)
          ,end="")
    #Calming down for inertial measurements
    sleep(4)
    return msg

#read specific yaw
def gotoyaw(connection,yaw, speed):
    msgl=rlocation(connection)
    msga=rattitude(connection)
    arr=np.array([msgl.x,msgl.y,msgl.z,msga.yaw])
    darr=np.array([0,0,0,yaw])
    connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, connection.target_system,
                        connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                            int(0b100111111000),arr[0],arr[1],arr[2],1,1,1,0,0,0,yaw,speed))
    connection.recv_match(type='COMMAND_ACK', blocking=True)
    print("Undergoing displacement")
    connection.recv_match(type='MISSION_ACK', blocking=True)
    msg = connection.recv_match(type='ATTITUDE', blocking=True)
    yaw=msg.yaw
    #Calming down for inertial measurements
    sleep(2)
    print("The current yaw has become (%.2f)"% (yaw))
    return msg
#reach specific altitude

def gotoyaw2(connection,yaw, speed, tol):
    msgl=rlocation(connection)
    msga=rattitude(connection)
    arr=np.array([msgl.x,msgl.y,msgl.z,msga.yaw])
    connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, connection.target_system,
                        connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                            int(0b100111111000),arr[0],arr[1],arr[2],1,1,1,0,0,0,yaw,speed))
    msg=connection.recv_match(type='ATTITUDE', blocking=True)
    curryaw=msg.yaw
    while(abs(yaw-curryaw) > tol):
        print("not there")
        msg=connection.recv_match(type='ATTITUDE', blocking=True)
        curryaw=msg.yaw
        pass
    #Calming down for inertial measurements
    sleep(4)
    print("The current yaw has become (%.2f)"% (curryaw))
    return msg

def gotoalt(connection, taralt, speed):
    msgl=rlocation(connection)
    msga=rattitude(connection)
    arr=np.array([msgl.x,msgl.y,msgl.z,msga.yaw])
    darr=np.array([0,0,0,0])
    #transforming difference array darr
    connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, connection.target_system,
                        connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                            int(0b100111111000),arr[0],arr[1],-taralt,0.25,0.25,speed,0,0,0,arr[3],0.25))
    connection.recv_match(type='COMMAND_ACK', blocking=True)
    print("Undergoing displacement")
    connection.recv_match(type='MISSION_ACK', blocking=True)
    #Calming down for inertial measurements
    sleep(2)
    msg = raltitude(connection)
    return msg

#reach specific altitude
def gotoalt2(connection, taralt, speed, tol):
    #know current location
    absalt=rlocation(connection)
    #know target alt
    msgl=rlocation(connection)
    msga=rattitude(connection)
    arr=np.array([msgl.x,msgl.y,msgl.z,msga.yaw])
    darr=np.array([0,0,0,0])
    #transforming difference array darr
    connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, connection.target_system,
                        connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                            int(0b100111111000),arr[0],arr[1],-taralt,0.25,0.25,speed,0,0,0,arr[3],0.25))
    msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    height=-msg.z
    while(abs(taralt-height) > tol):
        print("not there")
        msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        height=-msg.z
        pass
    #Calming down for inertial measurements
    sleep(4)
    print("The current height has become (%.2f)"% (height))
    return msg

#go GUIDED, see https://ardupilot.org/copter/docs/parameters.html#fltmode1
def setguided(connection):
    msg=connection.mav.command_long_send(connection.target_system,connection.target_component,
                                    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 4, 0, 0, 0, 0, 0, 0)
    return msg

#go AUTO, see https://ardupilot.org/copter/docs/parameters.html#fltmode1
def setauto(connection):
    msg=connection.mav.command_long_send(connection.target_system,connection.target_component,
                                    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 3, 0, 0, 0, 0, 0, 0)
    return msg

#go RTL, see https://ardupilot.org/copter/docs/parameters.html#fltmode1
def setrtl(connection):
    msg=connection.mav.command_long_send(connection.target_system,connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 6, 0, 0, 0, 0, 0, 0)
    return msg
#Land, see https://ardupilot.org/copter/docs/parameters.html#fltmode1
def setland(connection):
    msg=connection.mav.command_long_send(connection.target_system,connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 9, 0, 0, 0, 0, 0, 0)
    return msg
    
#RTL is not allowed in teknofest as per 
#https://cdn.teknofest.org/media/upload/userFormUpload/Published-7-UAV_COMPETITION_RULES_BOOKLET_2024_V1.15_ev9jf.pdf
#This is our own implementation
def manualtl(connection,speed,angspeed):
    height=raltitude(connection, height)
    #insert 0 0 with same height in NED frame
    msg=connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, connection.target_system,
                        connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                            int(0b100111111000),0,0,-height,speed,speed,speed,0,0,0,0,angspeed))
    msg=setland(connection)
    return msg
