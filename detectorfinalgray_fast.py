# Distingstion to detector1try2-2.py
#:: PREPROCESSING in GRAYSCALE to speedup, implementing masking and np.where() to substract background and then shadow_out for shadow(lighter ones and not the strong ones) removal
#:: Also Publishes the cropped image on a topic : 'Cropped'
#RESULTS: 3x faster, dt~(20ms-30ms)


import rclpy #add to package.xml deps
from rclpy.node import Node
import sensor_msgs
import std_msgs
from cv_bridge import CvBridge
import cv2
import numpy as np
import imutils
from imutils import perspective
from imutils import contours
from scipy.spatial import distance as dist
import pandas as pd
#import matplotlib.pyplot as plt

import time
import cProfile
import io
import pstats

def profile(func):
    def wrapper(*args, **kwargs):
        pr = cProfile.Profile()
        pr.enable()
        retval = func(*args, **kwargs)
        pr.disable()
        s = io.StringIO()
        sortby = SortKey.CUMULATIVE  # 'cumulative'
        ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
        ps.print_stats()
        print(s.getvalue())
        return retval

    return wrapper


#import commentjson as json
import pkg_resources
import argparse

from std_msgs.msg import String
from sensor_msgs.msg import Image

t1=0
y = 0
x = np.uint8(np.zeros((1080,1920)))
z = np.uint8(np.full((1080,1920), 255))


img_raw_sub = np.uint8(np.zeros((1080,1920)))
white = np.full((1080,1920), 255)
white = white.astype('uint8')

#@profile
def edgedetect(img, low, high, i):
  edged = cv2.Canny(img, low, high)
  edged = cv2.dilate(edged, None, iterations= i)
  edged = cv2.erode(edged, None, iterations= i)
  #edged = cv2.bitwise_not(edged);
  return edged

#@profile
def thres(img, th, maxVal, blocksize, C):
  img1 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
  _, img2 = cv2.threshold(img1, th, maxVal, cv2.THRESH_BINARY) 
  #img2 = cv2.adaptiveThreshold(img1, maxVal, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, blocksize, C)
  return img2

#@profile
def shadow_out(img, dil, bg_blur):

    dilated_img = cv2.dilate(img, np.ones((dil,dil), np.uint8))
    bg_img = cv2.medianBlur(dilated_img, bg_blur)
    diff_img = 255 - cv2.absdiff(img, bg_img)
    #norm_img = cv2.normalize(diff_img,None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
    return diff_img


def nothing(x):
    pass


class CrowVision(Node):

  def __init__(self):

    super().__init__('crow_detector')
    self.ros = {}
    ## handle multiple inputs (cameras).
    # store the ROS Listeners,Publishers in a dict{}, keys by topic.

    prefix = 'camera1'

    # create Input listener with raw images
    topic = 'color/image_raw'
    camera_topic = prefix + "/" + topic
    listener = self.create_subscription(msg_type=sensor_msgs.msg.Image,
                                          topic=camera_topic,
                                          # we're using the lambda here to pass additional(topic) arg to the listner. Which then calls a different Publisher for relevant topic.
                                          callback=lambda msg, topic=camera_topic: self.input_callback(msg, topic),
                                          qos_profile=1) #the listener QoS has to be =1, "keep last only".

    self.get_logger().info('Input listener created on topic: "%s"' % camera_topic)
    self.ros[camera_topic] = {} # camera_topic is used as an ID for this input, all I/O listeners,publishers will be based under that id.
    self.ros[camera_topic]["listener"] = listener

    self.cvb_ = CvBridge()

      #TODO others publishers

    self.publisher_ = self.create_publisher(Image, 'Cropped', 10)
    timer_period = 0.5  # seconds
    #self.timer = self.create_timer(timer_period, self.timer_callback)
    #self.i = 0


  #@profile(sort_by='cumulative', lines_to_print=10, strip_dirs=True)
  def timer_callbackX(self):
      msg = string()
      msg.data = 'Lets ROS: %d' % self.i
      self.publisher_.publish(msg)
      self.get_logger().info('Publishing: "%s"' % msg.data)       
      self.i += 1




  def input_callback(self, msg, topic):
    """
    @param msg - ROS msg (Image data) to be processed. From camera
    @param topic - str, from camera/input on given topic.
    @return no thing, but send new message(s) via output Publishers.
    """
    #t1 is the initial runtime for finding Execution time of this section
    t1=time.time()

    #Global variables
    global x, y, img_raw_sub


    self.get_logger().info("I heard: {} for topic {}".format(str(msg.height), topic))
    assert topic in self.ros, "We don't have registered listener for the topic {} !".format(topic)
    
    #trackbar consists of the variables which have a possibility to be tuned up over external factors
    #cv2.namedWindow('Trackbar')
    #if y == 0:
      #cv2.createTrackbar('BLUR','Trackbar',1,20,nothing)
      #cv2.createTrackbar('BLUR2','Trackbar',1,20,nothing)
      #cv2.createTrackbar('maskingThres','Trackbar',38,60,nothing)

      #cv2.createTrackbar('e_low','Trackbar',76,255,nothing)
      #cv2.createTrackbar('e_high','Trackbar',255,255,nothing)
      #cv2.createTrackbar('e_iter','Trackbar',8,20,nothing)

      #cv2.createTrackbar('cntrst','Trackbar',9,100,nothing) 

      #cv2.createTrackbar('S_dil_Krnl','Trackbar',5,20,nothing)
      #cv2.createTrackbar('S_bg_blur','Trackbar',1,20,nothing)

    #Assigning values to the variables based on the Slie bar's position
    BLUR = 3
    BLUR2 = 3
    mThres = 38

    e_low = 76
    e_high = 255
    e_iter = 8
    cntrst = (0.1)*9


    dil = 11
    bg_blur = 3
    th = np.full((1080,1920), mThres)

    #Image data from the realsense camera is translated into Numpy array format using CvBridge()
    img_raw_Color = self.cvb_.imgmsg_to_cv2(msg)
    img_raw = cv2.cvtColor(img_raw_Color, cv2.COLOR_BGR2GRAY)
    #cv2.imwrite('img_raw.jpg', img_raw)

    #Initial blurring for smoothening
    img_raw = cv2.medianBlur(img_raw, BLUR)
    #img_raw = img_raw.astype('uint8')318,
    #cv2.imshow('First blur', img_raw)

    #img2 is x2 is defined and set to int16 format to be able to be utilized for subtraction in the integer number line
    img2 = img_raw.astype('int16') 
    x2 =  x.astype('int16')

    #Substraction for initial background substraction
    #img_raw_sub = abs(img_raw2 - x2)
    img_sub = cntrst*cv2.absdiff(img2, x2)
    #cv2.imwrite('img_sub.jpg', img_sub)

    img2 = img_raw.astype('uint8')

    #Use Z for scaling blacks to whites
    #img_raw_sub = z - img_raw_sub

    #Masking out the object
    bg_sub1 = np.where(img_sub>th, img_raw, 0)
    bg_sub1 = cv2.medianBlur(bg_sub1, BLUR2)


    #saving background image in X
    if y <= 10:
      x = img_raw
      y=y+1  
      #cv2.imwrite('BG.jpg', x)

    summ = np.sum(np.sum(bg_sub1,axis = 0))
    print('summ', summ)
 
    if summ < 5000:
      t2=time.time()
      print('Breaking loop due to no significat detection, dT=',t2-t1)
      key = cv2.waitKey(10)
      # Press esc or 'q' to close the image window
      if key & 0xFF == ord('q') or key == 27:
        cv2.destroyAllWindows()
      return   

    #Shadow removal and smoothening
    bg_sub1 = shadow_out(bg_sub1, dil, bg_blur)


    edged1 = edgedetect(bg_sub1, e_low, e_high,e_iter)


    #Extracting contours out of the Edged image
    contours = cv2.findContours(edged1.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)

    print('contour lenght:', len(contours))
    l=len(contours)
    i = 1
    j = 0
    for contour in contours:
      area = cv2.contourArea(contour)
      print('area', area)

      if area>800:
        j = j+1
        a,b,w,h = cv2.boundingRect(contour)
        #draw the rect enclosing
        box_extended = box + [[-20 ,-20],[20, -20],[20, 20],[-20, 20]]
        cv2.drawContours(img_raw,[box_extended],0,(0,0,255),1)

        #Defining extra gap over the contours : bd
        gp=10

        cv2.rectangle(img_raw_Color, (a, b), (a+w, b+h), (0, 255, 0), 2)

        # compute the center of the contour
        M = cv2.moments(contour)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        #draw the contour and center of the shape on the image mentioning contour's count
        cv2.drawContours(img2, contours, -1, (0, 255, 0),1)  
        cv2.putText(img2, str(i), (cX, cY ), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        #let fixed ratio: H/W = r
        r = 1 # we can change as per requirements; r=H/W

        # Initialize arguments for the filter
        borderType = cv2.BORDER_CONSTANT


        if h>w:
          xx = int((((h + 2*gp) / r) - w )/ 2)
          Bordered_img = cv2.copyMakeBorder(img_raw_Color, xx, xx, xx, xx, borderType, None, [255,255,255])
          # Visulaization of the new coordinates and crop window
          #cv2.rectangle(Bordered_img, (a-xx +xx, b-gp +xx), ( a+w+xx +xx, b+h+gp +xx), (0, 0, 255), 2)
          #cv2.circle(Bordered_img, (a +xx,b +xx), radius=1, color=(255, 255, 255), thickness=2)
          #cv2.circle(Bordered_img, (a+w +xx,b+h +xx), radius=1, color=(0, 0, 0), thickness=2)

          cropped = Bordered_img[b-gp +xx : b+h+gp +xx , a-xx +xx : a+w+xx +xx]
          #cv2.imwrite('Bordered_img.jpg', Bordered_img)
        else:
          yy = int((((w + 2*gp) * r) - h )/ 2)
          Bordered_img = cv2.copyMakeBorder(img_raw_Color, yy, yy, yy, yy, borderType, None, [255,255,255])
          # Visulaization of the new coordinates and crop window
          #cv2.rectangle(Bordered_img, (a-gp +yy, b-yy +yy), (a+w+gp +yy, b+h+yy +yy), (0, 0, 255), 2)
          #cv2.circle(Bordered_img, (a +yy,b +yy), radius=1, color=(255, 255, 255), thickness=2)
          #cv2.circle(Bordered_img, (a+w +yy,b+h +yy), radius=1, color=(0, 0, 0), thickness=2)

          cropped = Bordered_img[b-yy +yy : b+h+yy +yy , a-gp +yy : a+w+gp +yy]
          #cv2.imwrite('Bordered_img.jpg', Bordered_img)


        #Publishing cropped image
        pub_msg = cropped
        #pub_msg.data = 'Cropped image: %d' % i
        self.publisher_.publish(self.cvb_.cv2_to_imgmsg(np.array(pub_msg)))
        self.get_logger().info('Publishing:')#"%s"' % pub_msg.data)       
        #self.i += 1


        i = i+1

    if i > 1:
      #cropped.tight_layout()
      #plt.show()
      cv2.imshow('Cropped image', cropped)
      #cv2.imwrite('cropped.jpg', cropped)



    #t2 is the final runtime for finding Execution time of this section
    t2=time.time()
    print('dT=',t2-t1)

    key = cv2.waitKey(10)
    # Press esc or 'q' to close the image window
    if key & 0xFF == ord('q') or key == 27:
    	cv2.destroyAllWindows()



def main(args=None):
  rclpy.init(args=args)
  
  crow_detector=CrowVision()
  #rclpy.spin_once(crow_detector)
  rclpy.spin(crow_detector)

  cv2.destroyAllWindows()
  rclpy.shutdown()


if __name__ == '__main__':
    main()

