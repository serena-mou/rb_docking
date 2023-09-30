# ros node to start on boot. Opens the serial line. Reads position data from floatyboat + log flag ($ASVPD, long, lat, heading, depth, log_flag, log_name)
# Writes serial rangerbot angle from usb_cam
#
# Written by:   Serena Mou
# Date:         29th September 2023

import numpy as np
import serial
import sys
import os
import datetime
import cv2
import time
import csv
import threading
import queue

import rospy
import std_msgs.msg
from std_msgs.msg import Int8, Float64, String
from sensor_msgs.msg import Image


class serialInOut():
    
    def __init__(self):

        self.ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)
        self.ser.flushInput()
        self.ser.flushOutput()
        
        self.sub_info = rospy.Subscriber('/rb_info', String, self.callback, queue_size=1)
        self.pub_serial = rospy.Publisher('/serial_out', String, queue_size=5)

        self.line = ''    

    def read_serial(self,ser,readQueue):
        #set up serial in
        while(True):
            pd_line = ser.readline()
            print("serial in",pd_line)
            if str(pd_line[0:6]) == "b'$ASVPD'" and pd_line[-2:] == "\r\n".encode('utf-8'):
                self.line = pd_line
                #self.callback()#readQueue.put(pd_line)

    def callback(self,rb_info):
        
        info = rb_info.data
        info = info.split(',')
        
        rb_angle = info[0]
        pix_from_bottom = info[1]
        area = info[2]
        filler = str('')
        NMEA_0 = 'RBHED'
        
        write_str = '%s,%s,%s,%s' % (NMEA_0,rb_angle, pix_from_bottom, area) 
        self.pub_serial.publish(write_str)
        print(write_str) 

        crc = 0
        for c in write_str:
            crc = crc ^ ord(c)
        crc = crc & 0xFF
        write_bytes = '$%s*%0.2X\r\n' % (write_str, crc)
        write_bytes = write_bytes.encode('utf-8')
        print("serial out", write_bytes)
        self.ser.write(write_bytes)


def main():
    s_io = serialInOut()
    readQueue = queue.Queue()
    thread = threading.Thread(target=s_io.read_serial, args=(s_io.ser,readQueue))
    rospy.init_node("serialInOut", disable_signals=True)
    try:
        thread.start()
        rospy.spin()
    except (KeyboardInterrupt, SystemExit):
        s_io.ser.close()
        print("shutting down serial/ROS node")
    cv2.destroyAllWindows()
    
    #s_io.callback(readQueue)

if __name__ == '__main__':
    main()

