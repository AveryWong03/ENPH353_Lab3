#!/usr/bin/env python3
from __future__ import print_function

import roslib
roslib.load_manifest('enph353_ros_lab')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
PTHRESH = 0
WIDTH,HEIGHT = 320,240
BLUR = 25
CANNY_LTHRESH = 50
CANNY_HTHRESH = 150
FROM_BOTTOM = 40

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Begin image processing
  
    # resize image
    resized = cv2.resize(cv_image, (WIDTH,HEIGHT))
    
    # blur image
    image_blur = cv2.GaussianBlur(resized,(BLUR,BLUR),0)
    edges = cv2.Canny(resized,CANNY_LTHRESH,CANNY_HTHRESH)  
    
    (rows,cols,channels) = resized.shape
    image_center = cols/2
 
    # Find first and last white pixel
    first = True
    firstp = -1
    lastp = -1
    for x in range(cols):
      if(edges[rows-FROM_BOTTOM][x] != 0):
        lastp = x
        if(first == True):
          firstp = x
          first = False
 
    # Setup move message
    pub = rospy.Publisher('/cmd_vel', Twist,queue_size=1)
    rate = rospy.Rate(2)
    move = Twist()

    center_coord = (lastp+firstp)/2
    p = center_coord - image_center

    # PID
    if(p > PTHRESH):
      # Turn right
      move.angular.z = -3
      move.linear.x = 0.5
    elif(p < -PTHRESH):
      # Turn left
      move.angular.z = 3
      move.linear.x = 0.5
    else:
      move.linear.x = 0.5
    
    
    pub.publish(move)
    cv2.imshow("Image window",edges)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)