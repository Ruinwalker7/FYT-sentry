// Copyright FYT 邹承甫 2022

#include "filters/kalman_filter.hpp"
namespace post_processor
{

KalmanFilter::KalmanFilter(
  const Eigen::MatrixXd &F,
  const Eigen::MatrixXd &P,
  const Eigen::MatrixXd &Q,
  const Eigen::MatrixXd &R,
  const Eigen::MatrixXd &H)
: F_(F), P_(P),P_init_(P), Q_(Q), R_(R), H_(H), chi_square_flag_(false)
{
}

void KalmanFilter::initialize(const Eigen::VectorXd &x0)
{
  x_ = x0;
  x_predict_ = x0;
  P_ = P_init_;
}

Eigen::VectorXd KalmanFilter::update(const Eigen::VectorXd &measurement, const double &dt)
{
  chi_square_flag_ = false;

  F_(0,3) = F_(1,4) = F_(2,5) = dt;
  x_predict_ = F_*x_;

  P_predict_ = F_*P_*F_.transpose() + Q_;

  Eigen::VectorXd residual = measurement - H_*x_;
  Eigen::MatrixXd CvK = H_*P_predict_*H_.transpose() + R_;
  // chi_square_flag_ = chiSquareTest(residual,CvK);
  chi_square_flag_ = normTest(residual);
  if(chi_square_flag_)
  {
    Eigen::VectorXd tmp_state = Eigen::VectorXd::Zero(7);
    tmp_state << measurement[0], measurement[1], measurement[2], 0, 0, 0, measurement[6];
    initialize(tmp_state);
    return tmp_state;
  }
  else
  {
    Eigen::MatrixXd K = P_predict_*H_.transpose()*(CvK.inverse());
    x_ = x_ + K*residual;
    P_ = P_predict_ - K*H_*P_predict_;
    x_predict_ = F_*x_;
    return x_;
  }
}

Eigen::VectorXd KalmanFilter::predict(const double &dt)
{
  F_(0,3) = F_(1,4) = F_(2,5) = dt;
  Eigen::VectorXd tmp = F_*x_;
  return tmp;
}

bool KalmanFilter::chiSquareTest(const Eigen::MatrixXd &residual, const Eigen::MatrixXd &covariance)
{
  Eigen::MatrixXd ak = residual.transpose()*covariance.inverse()*residual;  
  return ak(0) > 23.23;
}

bool KalmanFilter::normTest(const Eigen::MatrixXd &residual)
{
  double square_norm = residual.norm()/residual.size();
  return square_norm > 200;
}

}