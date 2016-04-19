#!/usr/bin/env python
from __future__ import print_function
PKG = 'sidewalk_detector'
import roslib
roslib.load_manifest(PKG)
import sys
import rospy
import cv2
import numpy as np
import random
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class detector:

  def __init__(self):
    self.image_pub = rospy.Publisher("/sidewalk_detector/color/image_raw",Image,queue_size=10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Detect sidewalk
    frame = cv2.flip(cv_image,0)
    blurred = cv2.GaussianBlur(frame,(5,5),0)
    frame_h,frame_w,ch = frame.shape
    ground = blurred[frame_h/2:frame_h,0:frame_w]
    ground_h,ground_w,ch = ground.shape
    # convert lower half of frame to HSV
    hsv_f = cv2.cvtColor(ground,cv2.COLOR_BGR2HSV)
    # define training area and convert to HSV
    train_w = 200
    train_h = 200
    train_lbx = ground_h - train_h
    train_ubx = ground_h
    train_lby = (ground_w-train_w)/2
    train_uby = (ground_w+train_w)/2
    train = ground[train_lbx:train_ubx,train_lby:train_uby]
    hsv_t = cv2.cvtColor(train,cv2.COLOR_BGR2HSV)
    # calculate training histogram and normalize
    train_hist = cv2.calcHist([hsv_t], [0,1], None, [180,256], [0,180,0,256])
    #train_hist[:] = [x/1920 for x in train_hist]
    cv2.normalize(train_hist,train_hist,0,255,cv2.NORM_MINMAX)
    # find 3000 random background pixels and calculate histogram
    hist = np.zeros((180,256),np.uint8)
    def randompixel(npixels):
        image = np.zeros((ground_h,ground_w,ch),np.uint8)
        for i in range(npixels):
            x,y = random.randint(1,ground_h-1), random.randint(1,ground_w-1)
            if not ((train_lbx<=x<=train_ubx) and (train_lby<=y<=train_uby)):
                image[x,y] = hsv_f[x,y]
                hist[hsv_f[x,y][0],hsv_f[x,y][1]]+=1
        return image,hist
    image,hist = randompixel(3000)
    background_hist = hist
    cv2.normalize(background_hist,background_hist,0,255,cv2.NORM_MINMAX)
    
    probability = np.zeros((ground_h,ground_w),np.float32)
    for i in range(ground_h):
        for j in range(ground_w):
            hue = hsv_f[i,j,0]
            saturation = hsv_f[i,j,1]
            S_hs = train_hist[hue,saturation]
            B_hs = background_hist[hue,saturation]
            if not (S_hs+B_hs==0):
                probability[i,j] = S_hs/(S_hs+B_hs)
    # create mask and inverse
    ret,thresh = cv2.threshold(probability,0.5,255,0)
    mask = np.zeros((frame_h,frame_w,ch), np.uint8)
    mask[frame_h-ground_h:frame_h,0:frame_w] = cv2.merge((thresh,thresh,thresh))
    # blank red image same size as frame
    red_image = np.zeros((frame_h,frame_w,ch), np.uint8)
    red_image[:] = (0,0,255)
    
    # find and fill in contours
    #gray = cv2.cvtColor(ground,cv2.COLOR_BGR2GRAY)
    #ret,thresh = cv2.threshold(gray,60,255,cv2.THRESH_BINARY)
    #contours,hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #mask2 = np.zeros((frame_h,frame_w,ch), np.uint8)
    #cv2.drawContours(mask2[frame_h-ground_h:frame_h,0:frame_w],contours,-1,(0,0,255),-1)
    #mask_final = cv2.bitwise_or(mask2,cv2.bitwise_and(mask,mask2))
    #mask_final = cv2.bitwise_or(mask,mask2)
#    edges = np.zeros((frame_h,frame_w),np.uint8)
#    cv2.Canny(blurred,1,50,edges,3,True)
 #    find lines
#    lines = cv2.HoughLines(edges,1,np.pi/180,100,100,10)[0]
#    for rho,theta in lines:
#        a = np.cos(theta)
#        b = np.sin(theta)
#        x0 = a*rho
#        y0 = b*rho
#        x1 = int(x0 + 1000*(-b))
#        y1 = int(y0 + 1000*(a))
#        x2 = int(x0 - 1000*(-b))
#        y2 = int(y0 - 1000*(a))
#        angle = np.arctan2(-b,a)*180/np.pi
#        if (50<=theta<=80 or 100<=theta<=130):
#            cv2.line(frame,(x1,y1),(x2,y2),(0,0,255),2)
    

    mask_final = mask
    mask_inv = cv2.bitwise_not(mask_final)
    
    # subtract mask area from original image
    res = cv2.bitwise_and(frame,mask_inv)
    # take mask area of red image
    res2 = cv2.bitwise_and(red_image,mask_final)
    # combine original image and mask
    final = cv2.add(res,res2)
    

    # Display image
    cv2.imshow("Image window", final)
    cv2.waitKey(3)

    # Publish to topic
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)


def main(args):
  rospy.init_node(PKG)
  try:
    sd = detector()
  except rospy.ROSInterruptException:
    pass
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
