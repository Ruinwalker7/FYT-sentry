// Copyright FYT 邹承甫 2022

#include <ros/ros.h>
#include "post_processor/processor_node.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "post_processor");
  post_processor::ProcessorNode node;
  while(ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}