#!/usr/bin/env python

# Copyright FYT 邹承甫 2022

import roslib
import rospy
from std_msgs.msg import Header
from rm_interfaces.msg import SerialReceiveMsg

def main():
  rospy.init_node("serial_node", anonymous=True)
  pub = rospy.Publisher("serial_receive",data_class=SerialReceiveMsg,queue_size=10)
  rospy.set_param("/enemy_color",0)
  rate = rospy.Rate(100)
  while(not rospy.is_shutdown()):
    msg = SerialReceiveMsg()
    msg.bullet_speed=10.0
    msg.enemy_color=1
    msg.yaw, msg.pitch = 0.0, 0.0
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
    rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
