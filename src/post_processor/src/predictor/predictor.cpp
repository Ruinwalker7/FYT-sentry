// Copyright FYT 邹承甫 2022

#include "post_processor/predictor.hpp"

namespace post_processor{

Predictor::Predictor(const double &air_fraction)
:air_fraction_(air_fraction), is_ok(false)
{
  bullet_speed_ = 25;
  pitch_ = 0;
  air_fraction_ = 0.01;
}

void Predictor::setBulletSpeed(const double &bullet_speed)
{
  bullet_speed_ = bullet_speed;
}

void Predictor::setPitch(const double &pitch)
{
  pitch_ = pitch;
}

float Predictor::predict(Eigen::VectorXd &state)
{
  is_ok = false;
  double vx = state[3];
  double vy = state[4];
  double vz = state[5];
  double height, height_predict, dh, height_iter, angle;
  height_iter = state[2];
  height = state[2];
  double distance = sqrt(state[0]*state[0] + state[1]*state[1]);
  double flying_time = 0;

  int interation = 20;
  for(int i=0; i<interation; ++i)
  {
    angle = atan2(height_iter,distance);
    if(!calcBulletModel(distance, angle, height_predict, flying_time))
    {
      double tmp_pitch = angle - pitch_;
      return tmp_pitch;
    }
    dh = height/1000.0 - height_predict;
    height_iter += dh*1000.0;
    // printf("dh=%f\n",height_predict);
    if(fabs(dh) < 0.001)
    {
      break;
    }
  }
  // printf("flying_time %f\n",flying_time);
  // printf("angle = %f\n",angle*180.0/3.14);
  state[0] += vx*flying_time;
  state[1] += vy*flying_time;
  state[2] += vz*flying_time;
  float pitch = (angle - pitch_)*180.0f/Pi;
  is_ok = true;
  return pitch;
}

bool Predictor::calcBulletModel(const double &distance, const double &angle, double &height_predict, double&flying_time)
{
  if(fabs(air_fraction_) > 1 || air_fraction_ == 0 || bullet_speed_ == 0 || fabs(angle) > Pi/6.0)
  {
    return false;
  }
  flying_time = (exp(distance/1000.0 * air_fraction_) - 1) / air_fraction_ / bullet_speed_ / cos(angle);
  height_predict = bullet_speed_*sin(angle)*flying_time - 0.5*GRAVITY_ACC*flying_time*flying_time;
  return true;
}


} //namespace post_processor