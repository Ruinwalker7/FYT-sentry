// Copyright FYT 邹承甫 2022

#include <ros/ros.h>
#include "detector/rm_detector_node.hpp"

int main(int argc, char** argv)
{
  ros::init(argc,argv,"detector_node");
  detector::DetectorNode node;
  while (ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}


