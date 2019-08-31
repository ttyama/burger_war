#!/usr/bin/env python                                                                        
# -*- coding: utf-8 -*-  
# https://demura.net/lecture/12469.html                                                                    

import sys
import rospy
import cv2
import numpy as np
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt	
import message_filters
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
import tf



class image_converter:
  def __init__(self):
    self.name = rospy.get_namespace().replace("/", "")
    self.scan_pub = rospy.Publisher("scan_topic", LaserScan, queue_size=1)
    self.pose_pub = rospy.Publisher("enemyPose", PoseStamped, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = message_filters.Subscriber("/{}/image_raw".format(self.name), Image)
    self.scan_sub = message_filters.Subscriber("/{}/scan".format(self.name), LaserScan)
    self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.scan_sub], 30, 0.5)
    self.ts.registerCallback(self.callback)

    self.__listener = tf.TransformListener()
  
  def nothing(x):
    pass

  def getRadFromDeg(self, deg):
    return deg/180.0*math.pi
  
  def generatePose(self, x, y, th, timestamp):
    goal = PoseStamped()
    goal.header.frame_id = "/{}/base_scan".format(self.name)
    goal.header.stamp = timestamp
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.z = math.sin(self.getRadFromDeg(th)/2)
    goal.pose.orientation.w = math.cos(self.getRadFromDeg(th)/2)

    return goal

  def callback(self,data,scan):
    a = list(scan.ranges[180:359]) + list(scan.ranges[0:179])
    #print("preprocess")
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # RGB表色系からHSV表色系に変換                                                           
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # しきい値の設定（ここでは赤を抽出）

    # マスク画像を生成
    mask = np.zeros(hsv_image[:,:,0].shape, dtype=np.uint8)
    #mask[((hsv_image[:,:,0] < H_max) | (hsv_image[:,:,0] > H_min)) & (hsv_image[:,:,1] > 128)] = 255
    mask[((hsv_image[:,:,0] < 20) | (hsv_image[:,:,0] > 200)) & (hsv_image[:,:,1] > 128)] = 255
    
    # 画像配列のビット毎の倫理席。マスク画像だけが抽出される。                               
    cv_image2  = cv2.bitwise_and(cv_image, cv_image, mask = mask)

    # RGBからグレースケールへ変換                                                            
    gray_image = cv2.cvtColor(cv_image2, cv2.COLOR_BGR2GRAY)
    cv_image3  = cv2.Canny(gray_image, 75, 40);
    cv_image3_BGR = cv2.cvtColor(cv_image3, cv2.COLOR_GRAY2BGR)
    
    #print("contours start")
    #輪郭抽出
    #ret,thresh = cv2.threshold(gray_image,127,255,0)
    #view_contours = cv2.getTrackbarPos('view_contours','Parameter')
    enemyPos = [0,0]
    scan_copy = scan
    image, contours, hierarchy = cv2.findContours(cv2.medianBlur(gray_image, 3),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) != 0:
      #img_rinkaku = cv2.drawContours(cv_image2, contours[view_contours], -1, (0,255,0), 3)
      img_rinkaku = cv2.drawContours(cv_image2, contours, -1, (0,255,0), 3)
      x,y,w,h = cv2.boundingRect(contours[0])
      rect_img = cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,255,0),2)
      #49.5度　-24.75 -> 24.75, 49.5/640 = 0.07734375
      AnglePerPixel = 0.07734375
      TgtAngle = sorted([int((320-x)*AnglePerPixel), int((320-(x+w))*AnglePerPixel)])
      #TgtAngle = [0,-2]
      size_expand = 2
      scan_copy.ranges = a[179 + TgtAngle[0] - size_expand :179 + TgtAngle[1] + size_expand]
      scan_copy.intensities = []
      scan_copy.angle_min = (TgtAngle[0]) * scan_copy.angle_increment
      enemyRange = min(scan_copy.ranges)
      enemyRangeIndex = scan_copy.ranges.index(enemyRange)
      enemyTheta = scan_copy.angle_min + enemyRangeIndex * scan_copy.angle_increment
      enemyPos = [enemyRange * np.cos(enemyTheta), enemyRange * np.sin(enemyTheta)]
      #print("-")
      #print(self.generatePose(enemyPos[0], enemyPos[1], 0, now))
      #print("-")
    else:
      img_rinkaku = cv_image2


    # ウインドウ表示
    im_v = cv2.vconcat([cv_image, cv_image2])
    im_v = cv2.vconcat([im_v, cv_image3_BGR])
    cv2.imshow("Image", cv2.resize(im_v, (0,0),fx=0.5,fy=0.5))
    cv2.waitKey(3)
    
    
    
    a = list(scan.ranges[180:359]) + list(scan.ranges[0:179])
    #scan_copy.ranges = tuple(a)
    #scan_copy.intensities = []
    #scan_copy.angle_min = -179 * scan_copy.angle_increment
    #print(scan_copy)
    self.scan_pub.publish(scan_copy)

    now = rospy.Time.now()
    self.__listener.waitForTransform("/{}/odom".format(self.name), "/{}/base_link".format(self.name), now, rospy.Duration(1.0))
    #position, quaternion = self.__listener.lookupTransform("red_bot/odom", "red_bot/base_link", now)
    #print(position, quaternion)
    #print(self.__listener.getLatestCommonTime("/{}/odom".format(self.name), "/{}/base_link".format(self.name)))
    p = self.__listener.transformPose("/{}/odom".format(self.name), self.generatePose(enemyPos[0], enemyPos[1], 0, now))

    self.pose_pub.publish(self.generatePose(enemyPos[0], enemyPos[1], 0, now))

    print("----------------------------")

def main(args):
  
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

