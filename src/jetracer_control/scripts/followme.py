#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time 
import serial 
import numpy as np 


def openPort(port,baud):
    ser = serial.Serial(
        port=port,
        baudrate=baud,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        rtscts= True,
        timeout=10)
    return ser

def read_distance(ser):
    while True:
        if ser.read() == b'\x59':
            if ser.read() == b'\x59':
                rest = ser.read(7)
                if len(rest) != 7:
                    continue  

                dist_l = rest[0]
                dist_h = rest[1]
                distance = dist_h * 256 + dist_l

                return distance 
            

def calibr(ser):
    i=0
    while i < 1:
        line = ser.readline()
        if len(line) >60:
            line = str(line)
            line = line.split(',')
            line = line[2]
            i+=1
    return int(line)


aoa_board = openPort('/dev/ttyUSB0', 1000000)
lidar =  openPort('/dev/ttyTHS1', 115200)

while True:
    distance_cm = read_distance(lidar)
 #   print(f"Distance: {distance_cm} cm")
    
    angle = calibr(aoa_board)
  #  print(f"Angle: {angle} degrees")
    if distance_cm < 500 and distance_cm > 300:
        print(f"Object detectect at {distance_cm} cm, angle: {angle} degrees")
        print(f"Moving formard")  
    elif distance_cm < 300: 
        print(f"Object too close at {distance_cm} cm, angle: {angle} degrees")
        print(f"Stopping")

    time.sleep(0.1) 


    #this is the logic a i understood about assignment itslef 
    # the code reads distance from a lidar sensor and angle from an aoa board
    # if the distance is between 300 and 500 cm, it prints a message to
    # move forward, if the distance is less than 300 cm, it prints a message
    # to stop. board is pretty inaccurate, so additional method of validation would be good. 
    # Also, data from lidar streaming continuously, so would be better to make lidar to stream 
    # data once a second or so. 