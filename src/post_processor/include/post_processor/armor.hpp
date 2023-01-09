// Copyright FYT 邹承甫 2022

#ifndef POST_PROCESSOR__ARMOR_HPP
#define POST_PROCESSOR__ARMOR_HPP

#include <Eigen/Core>
#include <std_msgs/Time.h>


namespace post_processor
{
typedef struct armor
{
  explicit armor(
    const Eigen::Vector3d p,
    const ros::Time t,
    const int i,
    const double angle=0
  ):position(p),stamp(t),id(i),yaw(angle){}
  Eigen::Vector3d position;
  // Eigen::Vector3d normal_vector;
  double yaw;
  ros::Time stamp;
  int id; 
}Armor;

}
#endif