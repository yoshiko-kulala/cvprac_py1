#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

def process_image(msg):
    pub = rospy.Publisher('cobo_point', Point, queue_size=10)
    try:
        bridge = CvBridge()
        orig = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow('raw_image', orig)
        hsvLower = np.array([4, 127, 102])
        hsvUpper = np.array([9, 230, 153])
        hsv = cv2.cvtColor(orig, cv2.COLOR_BGR2HSV)
        mask=cv2.inRange(hsv, hsvLower, hsvUpper)
        cv2.imshow('mask_image', mask)
        mu = cv2.moments(mask, True)
        x,y= int(mu["m10"]/mu["m00"]) , int(mu["m01"]/mu["m00"])
        cv2.circle(orig, (x,y), 15, 100, 2, 4)
        cv2.imshow('out_image', orig)

        vel=Point()
        vel.x=x
        vel.y=y
        vel.z=0
        pub.publish(vel)

        cv2.waitKey(1)
    except Exception as err:
        print err

def start_node():
    rospy.init_node('img_proc')
    rospy.loginfo('node started')
    rospy.Subscriber("usb_cam/image_raw", Image, process_image)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass