import cv2
import numpy as np
import glob


images = sorted(glob.glob('../../docking_data/fb/bag2/*'))

#fb_boxes = []
#big_contours = []
print(len(images))
for i, im in enumerate(images): 
    if i < 50:
        continue
    full_fov = 80 #degrees

    # get rb
    bgr_im = cv2.resize(cv2.imread(im), (0,0), fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    hsv_im = cv2.cvtColor(bgr_im, cv2.COLOR_BGR2HSV)
   
    xl,yl,wl,hl = 0,130,290,410 #x,y,w,h
    xr,yr,wr,hr = 670,130,290,410 #x,y,w,h
    #fb_roi = np.zeros(hsv_im.shape, dtype=np.int8)
    #fb_roi[yl:yl+hl,xl:xl+wl] = hsv_im[yl:yl+hl,xl:xl+wl]
    #fb_roi[yr:yr+hr,xr:xr+wr] = hsv_im[yr:yr+hr,xr:xr+wr]
    #rongy_hsv = [(12,0,220),(180,255,255)]
    left_roi = hsv_im[yl:yl+hl,xl:xl+wl] 
    right_roi = hsv_im[yr:yr+hr,xr:xr+wr]
    hsv_im[yl:yl+hl,xl:xl+wl] = np.zeros(left_roi.shape)  
    hsv_im[yr:yr+hr,xr:xr+wr] = np.zeros(right_roi.shape)  
    
    #rongy_hsv = [(0,0,200),(170,54,255)]
    rongy_hsv = [(12,26,175),(33,255,255)]
    blue_hsv = [(52,0,0,),(174,255,255)]
    yellow_hsv = [(17,70,0),(27,255,255)]
    panel_hsv = [(0,0,128),(114,255,255)]
    
    horizon = (960,40)
    rongy = cv2.inRange(hsv_im, rongy_hsv[0], rongy_hsv[1])

    #blue_thresh = cv2.inRange(hsv_im, blue_hsv[0], blue_hsv[1])
    #yellow_thresh = cv2.inRange(hsv_im, yellow_hsv[0], yellow_hsv[1])
    #panel_thresh = cv2.inRange(hsv_im, panel_hsv[0], panel_hsv[1])
    
    rb_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (20,20))
    water_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (35,35))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
   
    
    #fb = yellow_thresh# | panel_thresh
    #print(fb.shape)


    #fb = cv2.morphologyEx(fb, cv2.MORPH_CLOSE, kernel)
    #fb = cv2.dilate(fb,water_kernel,1)
    #rongy = cv2.morphologyEx(rongy, cv2.MORPH_CLOSE, rb_kernel)
    #rongy = cv2.morphologyEx(rongy, cv2.MORPH_OPEN, rb_kernel)
    rongy = cv2.erode(rongy,kernel,1)
    rongy = cv2.dilate(rongy,rb_kernel,1)
    #water = cv2.morphologyEx(blue_thresh, cv2.MORPH_CLOSE, water_kernel)
    
    #_, boat_contours, h1 = cv2.findContours(fb, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    rb_contour = [None]
    '''
    if i == 0:
        for con in boat_contours:
            area = cv2.contourArea(con)
        
            if area> 10000:
                #print(area)
                big_contours.append(con)
            for cont in big_contours:
                rect = cv2.minAreaRect(cont)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                fb_boxes.append([box])

    for box in big_contours:
        cv2.drawContours(bgr_im,box,0,(0,0,255),2)
    rongy = cv2.drawContours(rongy, big_contours, -1, 0, -1)
    '''
    rongy[0:horizon[0]][0:horizon[1]]=0
    _,rb_contours, h2 = cv2.findContours(rongy, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    max_rb_contour = 0
    for con in rb_contours:
        area = cv2.contourArea(con)
        if area>max_rb_contour and area > 400:
            max_area = area
            rb_contour[0] = con
            max_rb_contour = area
    print(max_rb_contour) 
    if rb_contour[0] is not None:
        M = cv2.moments(rb_contour[0])
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        angle = round((cX - 480)/12,2)
        cv2.putText(bgr_im, "pix from bottom: "+str(int(540-cY)), (500,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0),2)
        
        cv2.putText(bgr_im, "angle: "+str(angle)+" deg", (250,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0),2)
        cv2.drawContours(bgr_im, rb_contour, -1, (255,0,0),3)
        cv2.circle(bgr_im, (cX, cY), 3, (0,0,255), -1)
    
    #bgr_im = cv2.drawContours(bgr_im, big_contours, -1, (0,255,0),3) 
    print(str(i)+': ',im)
    while(1):
        #cv2.imshow('color',yellow_thresh)
        cv2.imshow('mask',bgr_im)
        k = cv2.waitKey(33)
        if k==32:
            break

cv2.destroyAllWindows()

#if i >60: 
    #    break
