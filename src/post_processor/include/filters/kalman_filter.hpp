// Copyright FYT 邹承甫 2022

#ifndef POST_PROCESSOR__KALMAN_FILTER_HPP
#define POST_PROCESSOR__KALMAN_FILTER_HPP

#include <Eigen/Core>
#include <Eigen/Dense>

namespace post_processor{

class KalmanFilter
{
public:
  explicit KalmanFilter(
    const Eigen::MatrixXd &F,
    const Eigen::MatrixXd &P,
    const Eigen::MatrixXd &Q,
    const Eigen::MatrixXd &R,
    const Eigen::MatrixXd &H
  );

public:
  void initialize(const Eigen::VectorXd &x0);
  Eigen::VectorXd update(const Eigen::VectorXd &measurement, const double &dt);
  Eigen::VectorXd predict(const double &dt);
private:
  bool chiSquareTest(const Eigen::MatrixXd &residual, const Eigen::MatrixXd &covariance);

  bool normTest(const Eigen::MatrixXd &residual);
private:
  bool chi_square_flag_;

  Eigen::MatrixXd F_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd H_;
  Eigen::MatrixXd P_predict_;
  Eigen::MatrixXd P_init_;
  Eigen::VectorXd x_;
  Eigen::VectorXd x_predict_; 
};


} //namespace post_processor

#endif  //POST_PROCESSOR__KALMAN_FILTER_HPP