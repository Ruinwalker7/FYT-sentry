#!/usr/bin/env python

# Copyright FYT 邹承甫 2022

import roslib
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

def main():
    cnt = 0
    rospy.init_node("video_plyaer", anonymous=True)
    rospy.set_param("/camera/camera_matrix",[ 1438.212, 0,  400,
    0,  1441.123,   300,
    0,  0,  1])
    rospy.set_param("/camera/dist_coeffs",[-0.1072,0.0566,-0.0011,-0.002,-0.7412])
    pub = rospy.Publisher("raw_img", Image, queue_size=1)
    bridge = CvBridge()
    rate = rospy.Rate(30)
    cap = cv2.VideoCapture("/home/zcf/csurm/csurm-RMUC2022/ood_blue.mp4")

    if cap.isOpened() == False:
        print("Error in opening video stream or file")
    while(cap.isOpened())and(not rospy.is_shutdown()):
        ret, frame = cap.read()
        if ret:
            if cnt < 0*24*(60*8+30):
                cnt+=1
                continue
            # cv2.imshow('Frame',frame)
            msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            header = Header()
            header.stamp = rospy.Time.now()
            msg.header = header
            pub.publish(msg)
            rate.sleep()
            if cv2.waitKey(20) & 0xFF == 27:
                break
        else:
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

