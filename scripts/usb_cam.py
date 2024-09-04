#!/usr/bin/env python3

import rospy
import cv2
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import datetime


class usb_cam():
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        #Check if the webcam is opened correctly
        if not self.cap.isOpened():
            raise IOError("Cannot open usbcam")
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/usb_cam/image', Image, queue_size=5)
        self.pub_scaled = rospy.Publisher('/usb_cam/scaled', Image, queue_size=5)
        self.Hz = 5
        self.prev_time = datetime.datetime.now()
    
    def get_image(self):
        ret, frame = self.cap.read()
        if frame is not None:
            curr_time = datetime.datetime.now()
            time_diff = curr_time-self.prev_time
            if time_diff.total_seconds() < 1/self.Hz:
                return
            self.prev_time = curr_time
            
            full =self.bridge.cv2_to_imgmsg(frame, 'bgr8') 
            scaled =self.bridge.cv2_to_imgmsg(cv2.resize(frame, None, fx=0.2, fy=0.2, interpolation=cv2.INTER_AREA),'bgr8') 
            self.pub.publish(full)
            self.pub_scaled.publish(scaled)
            rospy.loginfo("RB docking usb_cam publishing")
        #frame = cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        #cv2.imshow('Input', frame)
        #c = cv2.waitKey(0)

if __name__ == '__main__':
    node = usb_cam()
    rospy.init_node("usb_cam", anonymous=True, disable_signals=True)
    try:
        while True:
            node.get_image()
    except (KeyboardInterrupt, SystemExit):
        rospy.loginfo("RB docking USB_cam node killed")
        print('Killed')
        pass
