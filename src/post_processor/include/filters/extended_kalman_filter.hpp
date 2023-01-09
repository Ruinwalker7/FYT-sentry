// Copyright FYT 邹承甫 2022

#ifndef POST_PROCESSOR__EXTENDED_KALMAN_FILTER_HPP
#define POST_PROCESSOR__EXTENDED_KALMAN_FILTER_HPP

#include <Eigen/Core>
#include <Eigen/Dense>

namespace post_processor
{
/**
* 基于CTRV(Constant Turn Rate and Velocity)运动学模型的扩展卡尔曼滤波器
* 估计状态为{x, y, z, vx, vy, vz, theta, omega}
*/
class ExtendedKalmanFilter
{
public:
  explicit ExtendedKalmanFilter(
    const Eigen::MatrixXd &Q,
    const Eigen::MatrixXd &R,
    const Eigen::MatrixXd &P
  );
/**
 * @param measure x y z vx vy vz theta omega
 */
  Eigen::VectorXd update(const Eigen::VectorXd &measure, const double &dt);
  Eigen::VectorXd predict(const double &dt);
  void initialize(const Eigen::VectorXd &measure);
private:
  bool normTest(const Eigen::VectorXd &residual);

private:
  bool chi_square_flag_;

  Eigen::MatrixXd Jacobian_F_;
  Eigen::MatrixXd Jacobian_H_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd P_init_;
  Eigen::MatrixXd P_predict_;
  Eigen::VectorXd X_;
  Eigen::VectorXd X_predict_;
  Eigen::VectorXd variance_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
};

} //namespace post_processor

#endif //POST_PROCESSOR__EXTENDED_KALMAN_FILTER_HPP