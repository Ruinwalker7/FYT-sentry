// Copyright FYT 邹承甫 2022

#include "post_processor/pnp_solver.hpp"
namespace post_processor
{
PnPSolver::PnPSolver(
  const std::vector<float> &camera_matrix,
  const std::vector<float> &dist_coeffs
)
{
  camera_matrix_ = cv::Mat_<float>(3,3);
  utils::toCvMatrix(camera_matrix_,camera_matrix);
  dist_coeffs_ = cv::Mat_<float>(1,5);
  utils::toCvMatrix(dist_coeffs_,dist_coeffs);

  optical_point_.x = camera_matrix[2];
  optical_point_.y = camera_matrix[5];

  double small_armor_height = 85;
  double small_armor_width = 143;
  double big_armor_height = 85;
  double big_armor_width = 233;
  //tl bl br tr
  small_armor_pts_.emplace_back(cv::Point3f(-small_armor_width/2.0,small_armor_height/2.0,0));
  small_armor_pts_.emplace_back(cv::Point3f(-small_armor_width/2.0,-small_armor_height/2.0,0));
  small_armor_pts_.emplace_back(cv::Point3f(small_armor_width/2.0,-small_armor_height/2.0,0));
  small_armor_pts_.emplace_back(cv::Point3f(small_armor_width/2.0,small_armor_height/2.0,0));
  big_armor_pts_.emplace_back(cv::Point3f(-big_armor_width/2.0,big_armor_height/2.0,0));
  big_armor_pts_.emplace_back(cv::Point3f(-big_armor_width/2.0,-big_armor_height/2.0,0));
  big_armor_pts_.emplace_back(cv::Point3f(big_armor_width/2.0,-big_armor_height/2.0,0));
  big_armor_pts_.emplace_back(cv::Point3f(big_armor_width/2.0,big_armor_height/2.0,0));
}

Eigen::VectorXd PnPSolver::solvePnP(
  std::vector<cv::Point2f> &img_pts,
  const int &is_big
){
  CV_Assert(img_pts.size()==4);
  cv::Mat rvec;
  cv::Mat tvec;
  if(is_big)
  {
    cv::solvePnP(big_armor_pts_,img_pts,camera_matrix_,dist_coeffs_,rvec,tvec,false,cv::SOLVEPNP_IPPE);
  }
  else
  {
    cv::solvePnP(small_armor_pts_,img_pts,camera_matrix_,dist_coeffs_,rvec,tvec,false,cv::SOLVEPNP_IPPE);
  }
  cv::Mat rmat;
  // Eigen::Matrix3d rmat_eigen;
  cv::Mat z_vec(3,1,CV_64F);
  z_vec.at<double>(0,0) = 0;
  z_vec.at<double>(1,0) = 0;
  z_vec.at<double>(2,0) = -100;
  cv::Rodrigues(rvec,rmat);
  cv::Mat z_vec_c = rmat*z_vec;
  // cv::cv2eigen(rmat,rmat_eigen);
  Eigen::VectorXd pose = Eigen::VectorXd::Zero(6,1);
  // Eigen::Quaterniond quat(rmat_eigen);
  // Eigen::Quaterniond quat(1,0,0,0);

  pose << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2),
    z_vec_c.at<double>(0), z_vec_c.at<double>(1), z_vec_c.at<double>(2);
  return pose;
}


}