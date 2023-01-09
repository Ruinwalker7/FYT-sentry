#include "filters/extended_kalman_filter.hpp"
#include<iostream>
namespace post_processor
{

ExtendedKalmanFilter::ExtendedKalmanFilter(
    const Eigen::MatrixXd &Q,
    const Eigen::MatrixXd &R,
    const Eigen::MatrixXd &P
):Q_(Q), R_(R), P_(P), P_init_(P), chi_square_flag_(false)
{
  Jacobian_F_ = Eigen::MatrixXd::Identity(8,8);
  Jacobian_H_ = Eigen::MatrixXd::Identity(8,8);
}

void ExtendedKalmanFilter::initialize(const Eigen::VectorXd &measure)
{
  std::cout<<"measure=\n"<<measure.transpose()<<std::endl;
  X_ = measure;
  X_predict_ = measure;
  P_ = P_init_;
}

Eigen::VectorXd ExtendedKalmanFilter::update(const Eigen::VectorXd &measure, const double &dt)
{
  chi_square_flag_ = false;
std::cout<<"measure=\n"<<measure.transpose()<<std::endl;
  double x = X_[0];
  double y = X_[1];
  double z = X_[2];
  double vx = X_[3];
  double vy = X_[4];
  double vz = X_[5];
  double theta = X_[6];
  double omega = X_[7];
  if(omega == 0)
  {
    omega = 0.00001;
  }
//CTRV求偏导
  Jacobian_F_(0,3) = (-cos(omega*dt + theta) + cos(theta))/omega;
  Jacobian_F_(0,6) = vx*(sin(omega*dt + theta) - sin(theta))/omega;
  Jacobian_F_(0,7) = vx*dt*sin(omega*dt + theta)/omega - vx*(-cos(omega*dt + theta) + cos(theta))/omega/omega;
  Jacobian_F_(1,4) = (sin(omega*dt + theta) - sin(theta))/omega;
  Jacobian_F_(1,6) = vy*(cos(omega*dt + theta) - cos(theta))/omega;
  Jacobian_F_(1,7) = vy*dt*cos(omega*dt + theta)/omega - vy*((sin(omega*dt + theta) - sin(theta)))/omega/omega;
  Jacobian_F_(2,5) = dt;
  Jacobian_F_(6,7) = dt;


  X_predict_ = Jacobian_F_*X_;
  std::cout<<Jacobian_F_.size()<<" "<<Q_.size()<<" "<<P_.size()<<std::endl;
  P_predict_ = Jacobian_F_*P_*Jacobian_F_.transpose() + Q_;

  Eigen::VectorXd residual = Jacobian_H_*measure - X_predict_;
  Eigen::MatrixXd CvK = Jacobian_H_*P_predict_*Jacobian_H_.transpose() + R_;

  chi_square_flag_ = normTest(residual);

  if(chi_square_flag_)
  {
    Eigen::VectorXd tmp_state = Eigen::VectorXd::Zero(8);
    tmp_state << x, y, z, 0, 0, 0, theta, 0;
    initialize(tmp_state);
    return tmp_state;
  }
  else
  {
    Eigen::MatrixXd K = P_predict_*Jacobian_H_.transpose()*(CvK.inverse());
    X_ = X_ + K*residual;
    P_ = P_predict_ - K*Jacobian_H_*P_predict_;
    return X_;
  }
}

bool ExtendedKalmanFilter::normTest(const Eigen::VectorXd &residual)
{
  double square_norm = residual.head(3).norm()/3;
  return square_norm > 200;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const double &dt)
{
  double x = X_[0];
  double y = X_[1];
  double z = X_[2];
  double vx = X_[3];
  double vy = X_[4];
  double vz = X_[5];
  double theta = X_[6];
  double omega = X_[7];
  if(omega == 0)
  {
    omega = 0.00001;
  }
//CTRV求偏导
  Jacobian_F_(0,3) = (-cos(omega*dt + theta) + cos(theta))/omega;
  Jacobian_F_(0,6) = vx*(sin(omega*dt + theta) - sin(theta))/omega;
  Jacobian_F_(0,7) = vx*dt*sin(omega*dt + theta)/omega - vx*(-cos(omega*dt + theta) + cos(theta))/omega/omega;
  Jacobian_F_(1,4) = (sin(omega*dt + theta) - sin(theta))/omega;
  Jacobian_F_(1,6) = vy*(cos(omega*dt + theta) - cos(theta))/omega;
  Jacobian_F_(1,7) = vy*dt*cos(omega*dt + theta)/omega - vy*((sin(omega*dt + theta) - sin(theta)))/omega/omega;
  Jacobian_F_(2,5) = dt;
  Jacobian_F_(6,7) = dt;

  return Jacobian_F_*X_;
}

}// namespace post_processor