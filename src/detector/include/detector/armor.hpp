#ifndef DETECTOR__ARMOR_HPP
#define DETECTOR__ARMOR_HPP

#include <opencv2/opencv.hpp>

namespace detector
{
struct Armor
{
  int id;
  cv::Rect box;
  std::vector<cv::Point2f> points;
};
}

#endif  //DETECTOR_ARMOR_HPP