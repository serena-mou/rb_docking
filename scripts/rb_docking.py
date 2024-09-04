#!/usr/bin/env python3

# Subscribe to the mounted usb-cam on FB frame
# Find RB if it is in frame and publish the angle at which the RB is relative to the FB
# Publish /rb_info in the format angle, pix from bottom, area
# Written by:   Serena Mou
# Date:         29 September 2023


import rospy
import cv2
import numpy as np
import glob
from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge


class RBAngle():
    def __init__(self):
        rospy.loginfo('init RB CV node')
        self.sub_usb_im = rospy.Subscriber('/usb_cam/image', Image, self.callback, queue_size=1)
        self.pub = rospy.Publisher('/rb_info', String, queue_size=5)
        self.pub_im = rospy.Publisher('/rb_info/image', Image, queue_size=5)
        self.pub_im_scaled = rospy.Publisher('/rb_info/image_scaled', Image, queue_size=5)
        self.bridge = CvBridge()
        
        self.full_fov = 80 #degrees       
        self.img_scale = 0.5

        self.SHOW = False
    def callback(self, im):
        # get the subscribed image and convert to cv2
        angle = -1
        pix_from_bottom = -1

 
        image_cv = self.bridge.imgmsg_to_cv2(im, desired_encoding='passthrough')
        
        #print('got image')
        bgr_im = cv2.resize(image_cv, (0,0), fx=self.img_scale, fy=self.img_scale, interpolation=cv2.INTER_AREA)
        hsv_im = cv2.cvtColor(bgr_im, cv2.COLOR_BGR2HSV)

        # left and right floatyboat arm mask out box 

        #xl,yl,wl,hl = 0,130,290,410 #x,y,w,h
        #xr,yr,wr,hr = 670,130,290,410 #x,y,w,h

        xl,yl,wl,hl = 0,174,226,366 #x,y,w,h
        xr,yr,wr,hr = 737,174,226,366, #x,y,w,h
        area_threshold = 360
        left_roi = hsv_im[yl:yl+hl,xl:xl+wl] 
        right_roi = hsv_im[yr:yr+hr,xr:xr+wr]
        hsv_im[yl:yl+hl,xl:xl+wl] = np.zeros(left_roi.shape)  
        hsv_im[yr:yr+hr,xr:xr+wr] = np.zeros(right_roi.shape) 
           
        horizon = (540,0)
        # HSV values for yellow/RB
        rongy_hsv = [(14,52,102),(48,255,255)]
        rongy = cv2.inRange(hsv_im, rongy_hsv[0], rongy_hsv[1])
        rongy[0:horizon[0]][0:horizon[1]]=0
       
        rb_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
       
        rongy = cv2.erode(rongy,kernel,1)
        rongy = cv2.dilate(rongy,rb_kernel,1)
        
        rb_contour = [None]

        rb_contours, _ = cv2.findContours(rongy, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        max_rb_contour = -1
        for con in rb_contours:
            area = cv2.contourArea(con)
            if area>max_rb_contour and area > area_threshold:
                max_area = area
                rb_contour[0] = con
                max_rb_contour = area
        
        if rb_contour[0] is not None:
            M = cv2.moments(rb_contour[0])
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            angle = round((cX - 480)/12,2)
            pix_from_bottom = int(540-cY)
            cv2.putText(bgr_im, "angle: "+str(angle)+" deg", (250,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0),2)

            cv2.putText(bgr_im, "pix from bottom: "+str(pix_from_bottom), (500,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0),2)
            
            cv2.drawContours(bgr_im, rb_contour, -1, (255,0,0),3)
            cv2.circle(bgr_im, (cX, cY), 3, (0,0,255), -1)
        
        pub_str = '%s,%s,%s' % (str(angle), str(pix_from_bottom), str(max_rb_contour))
        self.pub.publish(pub_str)
        self.pub_im.publish(self.bridge.cv2_to_imgmsg(bgr_im, 'bgr8'))
        self.pub_im_scaled.publish(self.bridge.cv2_to_imgmsg(cv2.resize(bgr_im,(0,0),fx=0.4,fy=0.4), 'bgr8'))
        
        if self.SHOW:
            while(1):
                cv2.imshow('color',bgr_im)
                cv2.imshow('mask',rongy)
                k = cv2.waitKey(33)
                if k==32:
                    break



def main():
    dock = RBAngle()
    rospy.init_node("rb_angle", disable_signals=True)
    try:
        rospy.spin()
    except (KeyboardInterrupt, SystemExit):
        rospy.loginfo("shutting down rb angle node")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
