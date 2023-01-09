// Copyright FYT 邹承甫 2022

#ifndef DETECTOR__DETECTOR_NODE_HPP
#define DETECTOR__DETECTOR_NODE_HPP

#include <memory>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "rm_interfaces/Armor.h"
#include "rm_interfaces/AllTargets.h"
#include "rm_interfaces/TrackInfo.h"

#include "../../utils/include/utils.hpp"
#include "armor.hpp"
//********ONNX请打开这里
// #include "onnxdetector.hpp"
#include "detector.hpp"

namespace detector{

class DetectorNode{

public:
  explicit DetectorNode();

private:
  void image_cb(const sensor_msgs::ImageConstPtr &msg);

  void track_info_cb(const rm_interfaces::TrackInfoConstPtr &msg);

  bool detect4pts(
    cv::Mat &roi, 
    std::vector<cv::Point2f> &img_points,
    const cv::Point2f &origin_pt);

private:
  ros::NodeHandle nh_;
  ros::Subscriber img_sub_;
  ros::Subscriber track_info_sub_;
  ros::Publisher targets_pub_;
  std::unique_ptr<Detector> detector_;
  //**********ONNX请打开这里
  // std::unique_ptr<onnxDetector> onnx_detector_;

  rm_interfaces::TrackInfo track_info_;
  bool debug_;
  int enemy_color_;
  cv::Rect interest_area_;
};

} //namespace detector

#endif
