// Copyright FYT 邹承甫 2022

#ifndef POST_PROCESSOR__PNP_SOLVER_HPP
#define POST_PROCESSOR__PNP_SOLVER_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "../../utils/include/utils.hpp"

namespace post_processor
{

class PnPSolver
{
public:
  explicit PnPSolver(
    const std::vector<float> &camera_matrix,
    const std::vector<float> &dist_coeffs
  );
  
public:
  Eigen::VectorXd solvePnP(
    std::vector<cv::Point2f> &img_pts,
    const int &is_big
    );

  inline void setOpticalCenter(const cv::Point2f &optical_offset)
  {
    camera_matrix_.at<float>(0,2) = optical_point_.x + optical_offset.x;
    camera_matrix_.at<float>(1,2) = optical_point_.y + optical_offset.y;
  }

private:
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  cv::Point2f optical_point_;
  std::vector<cv::Point3f> small_armor_pts_;
  std::vector<cv::Point3f> big_armor_pts_;
};

} //namespace post_processor
#endif