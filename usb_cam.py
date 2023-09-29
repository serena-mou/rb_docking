import rospy
import cv2
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import datetime

Hz = 4
prev_time = datetime.datetime.now()


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
        print('publishing to /usb_cam/image')
        self.pub_rb_status = rospy.Publisher('/usb_cam/rb_status', String, queue_size=5)
    
    def get_image(self):
        ret, frame = self.cap.read()
        if frame is not None:
            curr_time = datetime.datetime.now()
            time_diff = curr_time-prev_time
            if time_diff.total_seconds() < 1/Hz:
                return
            prev_time = curr_time
            
            full =self.bridge.cv2_to_imgmsg(frame, 'bgr8') 
            scaled =self.bridge.cv2_to_imgmsg(cv2.resize(frame, None, fx=0.2, fy=0.2, interpolation=cv2.INTER_AREA),'bgr8') 
            self.pub.publish(full)
            self.pub_scaled.publish(scaled)
            self.pub_rb_status.publish('RB_STATUS_HERE')

        #frame = cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        #cv2.imshow('Input', frame)
        #c = cv2.waitKey(0)

if __name__ == '__main__':
    rospy.init_node("usb_cam", anonymous=True, disable_signals=True)
    node = usb_cam()
    try:
        print('usb cam node started')
        while True:
            node.get_image()
    
    except KeyboardInterrupt:
        print('Killed')
        pass
