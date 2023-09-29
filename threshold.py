import cv2
import numpy as np 
import glob

all_ims = sorted(glob.glob('./29-09-23/bag1/*'))
#in_ims = ['./bag2/2609230925231227.jpg','./bag2/2609230925160057.jpg','./bag2/2609230925160057.jpg', './bag2/2609230925231227.jpg', './bag2/2609230925243286.jpg', './bag2/2609230925264219.jpg','./bag2/2609230925228746.jpg','./bag2/2609230925243286.jpg']
in_im = all_ims[147]#'./bag2/2609230925264219.jpg'#'./bag2/2609230925160057.jpg'

image = cv2.imread(in_im)
image = cv2.resize(image, (0,0), fx=0.6, fy=0.6, interpolation=cv2.INTER_AREA)
def callback(x):
    pass

def trackbars():
    cv2.namedWindow('image')

    ilowH = 0
    ihighH = 179

    ilowS = 0
    ihighS = 255
    ilowV = 0
    ihighV = 255

    # create trackbars for color change
    cv2.createTrackbar('lowH','image',ilowH,179,callback)
    cv2.createTrackbar('highH','image',ihighH,179,callback)

    cv2.createTrackbar('lowS','image',ilowS,255,callback)
    cv2.createTrackbar('highS','image',ihighS,255,callback)

    cv2.createTrackbar('lowV','image',ilowV,255,callback)
    cv2.createTrackbar('highV','image',ihighV,255,callback)



    while True:
        # grab the frame
        frame = image

        # get trackbar positions
        ilowH = cv2.getTrackbarPos('lowH', 'image')
        ihighH = cv2.getTrackbarPos('highH', 'image')
        ilowS = cv2.getTrackbarPos('lowS', 'image')
        ihighS = cv2.getTrackbarPos('highS', 'image')
        ilowV = cv2.getTrackbarPos('lowV', 'image')
        ihighV = cv2.getTrackbarPos('highV', 'image')

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_hsv = np.array([ilowH, ilowS, ilowV])
        higher_hsv = np.array([ihighH, ihighS, ihighV])
        mask = cv2.inRange(hsv, lower_hsv, higher_hsv)

        frame = cv2.bitwise_and(frame, frame, mask=mask)

        # show thresholded image
        cv2.imshow('image', frame)
        k = cv2.waitKey(1000) & 0xFF # large wait time to remove freezing
        if k == 113 or k == 27:
            break
    cv2.destroyAllWindows()

trackbars()
