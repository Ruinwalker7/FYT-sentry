// Copyright FYT 邹承甫 2022

#ifndef POST_PROCESSOR__PREDICTOR_HPP
#define POST_PROCESSOR__PREDICTOR_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace post_processor
{

const double GRAVITY_ACC = 9.7803;
const double Pi = 3.1415926535;

class Predictor
{
public:
  explicit Predictor(const double &air_fraction);
/**
 * @brief 弹道模型预测
 * @param target_state {x,y,z,vx,vy,vz} 
 * @return 俯仰角的补偿，预测后的坐标
 */
  float predict(Eigen::VectorXd &target_state);

  void setBulletSpeed(const double &bullet_speed);
  void setPitch(const double& pitch);

  bool is_ok;
private:
  bool calcBulletModel(const double &distance, const double &angle, double &height_predict, double&flying_time);

  double bullet_speed_;
  double pitch_;
  double air_fraction_;
}; 


}

#endif