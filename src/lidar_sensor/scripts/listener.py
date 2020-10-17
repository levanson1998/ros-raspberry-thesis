#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

# auth: Le Van Son
# mail: vanson1243@gmail.com

import rospy
import math
import numpy as np
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import LaserScan
import socket
import sys
import time
import serial
from threading import Thread, Timer
import os
import struct
import io

def DEG2RAD(deg):
    return deg*math.pi/180

def RAD2DEG(rad):
    return rad*180/math.pi

#-----------------------TCP/IP---------------------------
# Client
def connect():
    global sock
    TCP_IP = '192.168.0.105'
    TCP_PORT = 64405
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print("Connecting to {}:{}".format(TCP_IP, TCP_PORT))
    try:
        sock.connect((TCP_IP, TCP_PORT))
        print("Connect successfull !")
    except:
        print("Fail Connected !\n")
        return False
    return True

def sendCommand(cmd):
    global sock
    # print("Sending: {}".format(cmd))
    try:
        sock.sendall(cmd)
        # print("Send successing\n")
    except:
        print("Fail send !\n")

#-----------------------TCP/IP---------------------------

#-----------------------BEGIN SERIAL---------------------------
time_t = 0.1
def serialInit():
    global ser, sio

    ser = serial.Serial(
    port = '/dev/ttyAMA0', # /dev/ttyAMA0 /dev/ttyUSB0
    baudrate = 115200,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout = 1
    )
    # sio=io.TextIOWrapper(io.BufferedRWPair(ser, ser), encoding="utf-16", errors='ignore')
    # sio.flush()

next_call = time.time()

def transmitSerial():
    global next_call, time_t, ser
    '''
    a=12.34
    isStart = 0
    SttSpeed = 0

# ---> van toc banh trai, phai (12,34)
# ---< goc. gia toc tu cam bien mpu6050 ()
    a1 = int(a)
    a2 = int(round((a-a1)*10000))
    
    dataa1=a1.to_bytes(2, byteorder = "little", signed = True)
    a1L=dataa1[1]
    a1H=dataa1[0]
    dataa2=a2.to_bytes(2, byteorder = "little", signed = True)
    a2L=dataa2[1]
    a2H=dataa2[0]

    packet = [a1L, a1H, a2L, a2H, isStart,SttSpeed]
    # print(packet)
    # print("{} * {} * {} * {} * {}".format(a, a1, a2, isStart,SttSpeed))
    #print("time: ",time.time())
    # packet = [chr(dataIsTracking), chr(dataX1), chr(dataX2), chr(dataY1), chr(dataY2)]
    # print("{} - - {}".format(packet, type(packet)))

    # ser.write(packet)
    '''
    try:
        receiveData = ser.read(11)
        print("receive serial: ", receiveData)
    except:
        print("serial no data !!!")
    
    # if(len(receiveData)!= 0):
    #     speedCurrent=receiveData[0]+receiveData[1]/100
    #     ReStop = receiveData[2]
    # t+=1
    
    next_call = next_call + time_t
    # Timer = threading.Timer
    Timer( next_call - time.time(), transmitSerial ).start()



#-----------------------END SERIAL---------------------------
t2=0
#------------------BEGIN CALLBACK DATA LIDAR-------------------
def scanCallback(scan):
    global t2
    count = scan.scan_time/scan.time_increment
    t1=time.time()
    print("time callback: ", t1-t2)
    t2=time.time()

    ser.flushInput()
    if ser.inWaiting()>1:
        print("CAUTION BUFFER SERIAL ", ser.inWaiting())
    receiveData = ser.readline()
    # print("receiveData: ", receiveData)
    
    # print("in waiting: ", ser.inWaiting())

    '''
    doc du lieu serial, them vao bien data va sendCommand()
    danh gia toc do doc du lieu cua serial

    serial receive: imu(6), encoderL, encoderR
    '''
    data = bytearray()
    for x in scan.ranges:
        data.extend(bytearray(struct.pack("f", x)))
    data.extend(receiveData)
    # print("size buffer: {}".format(len(data)))
    sendCommand(data)
    '''
    truyen di du lieu lidar va imu
    nhan ve toc do v_l, v_r, start, stop,...

    goi serial xuong stm 
    '''


#------------------END CALLBACK DATA LIDAR-------------------

def main():
    #-----SERIAL INIT-------
    serialInit()   
    # transmitSerial()
    #-----SERIAL INIT-------

    global connect
    rospy.init_node('listener', anonymous=True)
    # print(sys.path)
    print(int.from_bytes(b'0x23', "big"))
    for i in range(5):
        if connect(): break
        time.sleep(2)
        if i == 4:
            sys.exit("Fail connect over 5 times !\n")
    
    rospy.Subscriber('scan', LaserScan, scanCallback, queue_size=1000)
    # value = (234.456, 1.01, 123.546, 23.86)
    # data = bytearray()

    # for x in value:
    #     data.extend(bytearray(struct.pack("f", x)))
    # while True:
    #     sendCommand(data)
    #     time.sleep(1)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
