// Copyright FYT 邹承甫 2022

#include "post_processor/tracker.hpp"

namespace post_processor
{

Tracker::Tracker(
  const std::vector<double>& Qvec,
  const std::vector<double>& Pvec,
  const std::vector<double>& Fvec,
  const std::vector<double>& Hvec,
  const std::vector<double>& Rvec,
  const bool& is_debug)
:debug_(is_debug)
{
  last_time = ros::Time::now();
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(7,7);
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(7,7);
  Eigen::MatrixXd F = Eigen::MatrixXd::Zero(7,7);
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(7,7);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(7,7);
  utils::toEigenMatrix(Q,Qvec);
  utils::toEigenMatrix(P,Pvec);
  utils::toEigenMatrix(F,Fvec);
  utils::toEigenMatrix(H,Hvec);
  utils::toEigenMatrix(R,Rvec);
#ifdef KALMAN
  kf_ = std::make_unique<KalmanFilter>(F,P,Q,R,H);
#endif
#ifdef PARTICLE
  pf_ = std::make_unique<ParticleFilter>(500, 3, Q.block(3,3,3,3), R.block(3,3,3,3), last_time.sec); 
#endif
#ifdef EXTENDED_KALMAN
  ekf_ = std::make_unique<ExtendedKalmanFilter>(Q,R,P);
#endif
  max_match_thresh_ = 500;
  max_queue_size_ = 5;
  lost_cnt_ = 0;
  tracking_id_ = 0;
#ifdef KALMAN
  last_state = Eigen::VectorXd::Zero(7);
#endif
#ifdef EXTENDED_KALMAN
  last_state = Eigen::VectorXd::Zero(8);
#endif
  state_ = tracker::state::LOST;
}

void Tracker::initialize(const std::vector<Armor> &targets)
{

  Armor selected_armor = targets[0];
  double min_distance = 10e5;
  for(auto &armor : targets)
  {
    double distance = armor.position.norm();
    if(distance < min_distance)
    {
      selected_armor = armor;
      min_distance = distance;  
    }
  }
  history_queue_.clear();
#ifdef KALMAN
  Eigen::VectorXd state = Eigen::VectorXd::Zero(7);
  state << selected_armor.position(0), selected_armor.position(1), selected_armor.position(2),
  0, 0, 0, selected_armor.yaw;
#endif
#ifdef EXTENDED_KALMAN
  Eigen::VectorXd state = Eigen::VectorXd::Zero(8);
  state << selected_armor.position(0), selected_armor.position(1), selected_armor.position(2),
  0, 0, 0,selected_armor.yaw,0;
#endif
#ifdef KALMAN
  kf_->initialize(state);
#endif
#ifdef EXTENDED_KALMAN
  ekf_->initialize(state);
#endif
#ifdef PARTICLE
  // pf_->initialize(Eigen::Vector4d::Zero(4));
#endif
  lost_cnt_ = 0;
  state_ = tracker::TRACKING;
  tracking_id_ = selected_armor.id;
  history_queue_.push_back(selected_armor);
  last_time = ros::Time::now();
  last_state = Eigen::VectorXd::Zero(7);
}

Eigen::VectorXd Tracker::update(const std::vector<Armor> &targets)
{
  //掉帧处理
  if(targets.empty())
  {
    Eigen::VectorXd target_state = Eigen::VectorXd::Zero(8);
    if(this->state_ == tracker::state::TRACKING)
    {
      lost_cnt_++;
      if(history_queue_.size()<2 || lost_cnt_>5)
      {
        this->state_ = tracker::state::LOST;
      }
      double dt = (history_queue_.back().stamp - history_queue_.front().stamp).toSec()/history_queue_.size();
#ifdef KALMAN
      Eigen::VectorXd target_state = kf_->predict(dt*lost_cnt_);
#endif
#ifdef EXTENDED_KALMAN
      Eigen::VectorXd target_state = ekf_->predict(dt*lost_cnt_);
#endif
      // Eigen::VectorXd target_state = pf_->predict(dt);
      return target_state;
    }
    else
    {
      return target_state;
    }
  }
  //追踪间隔太长
  double dt = (targets[0].stamp - last_time).toSec();
  last_time = targets[0].stamp;
  if(state_ == tracker::state::LOST || dt > 1.0)
  {
    initialize(targets);
#ifdef KALMAN
    Eigen::VectorXd target_state = kf_->predict(dt*lost_cnt_);
#endif
#ifdef EXTENDED_KALMAN
    Eigen::VectorXd target_state = ekf_->predict(0);
#endif
    // Eigen::VectorXd state = pf_->predict(0);
    return target_state;
  }
  else
  { 
    //优先选相同id的装甲板
    std::vector<Armor> same_id_armors;
    for(auto &target : targets)
    {
      if(target.id != tracking_id_)
        continue;
      same_id_armors.push_back(target);
    }
    if(!same_id_armors.empty())
    {
      lost_cnt_ = 0;
      bool matched = false;
      bool is_rotting = false;
      double min_diff = 10e9;
      Armor matched_armor = same_id_armors[0];
      for(auto &target : same_id_armors)
      {
#ifdef KALMAN
        Eigen::VectorXd predict_pos = kf_->predict(dt);
#endif
#ifdef EXTENDED_KALMAN
        Eigen::VectorXd predict_pos = ekf_->predict(dt);
#endif
        // Eigen::VectorXd predict_pos = pf_->predict(dt);

        double diff = (predict_pos.head(3) - target.position).norm();
        if(diff < min_diff)
        {
          matched_armor = target;
          min_diff = diff;
        }
      }
      if(min_diff < max_match_thresh_)
      {
        //如果视野里有不止一个相同id的目标而且当前选择目标旋转角度已经大于30度了，
        //认为目标在小陀螺而且我们选择了快要离开视野的那一块，重新选择
        if(fabs(matched_armor.yaw) > 30 && same_id_armors.size()!=1)
        {
          is_rotting = true;
        }
        else
        {
        matched = true;
        }
      }
      //如果有匹配的装甲板，更新队列，进行卡尔曼滤波
      if(matched)
      {
        double total_time = (matched_armor.stamp- history_queue_.front().stamp).toSec();
        Eigen::Vector3d velocity = (matched_armor.position - history_queue_.front().position)/total_time;
        if(history_queue_.size() < 2)
        {
          if(last_state[3] != 0)
          {
            velocity = 0.5*velocity + 0.5*last_state.block(0,0,3,3);
          }
        }
#ifdef EXTENDED_KALMAN
        double angle_velocity = (matched_armor.yaw - history_queue_.front().yaw)/total_time;
#endif

#ifdef PARTICLE
        velocity = pf_->update(velocity, 0);
#endif
#ifdef KALMAN
        Eigen::VectorXd measurement = Eigen::VectorXd::Zero(7);
        measurement << matched_armor.position[0], matched_armor.position[1], matched_armor.position[2], 
        velocity[0], velocity[1], velocity[2], matched_armor.yaw;
        Eigen::VectorXd state = kf_->update(measurement,dt);
#endif
#ifdef EXTENDED_KALMAN
        Eigen::VectorXd measurement = Eigen::VectorXd::Zero(8);
        measurement << matched_armor.position[0], matched_armor.position[1], matched_armor.position[2], 
        velocity[0], velocity[1], velocity[2], matched_armor.yaw, angle_velocity;
        Eigen::VectorXd state = ekf_->update(measurement,dt);
#endif
        history_queue_.push_back(Armor(state.head(3),matched_armor.stamp,matched_armor.id, state(6)));
        if(history_queue_.size() > max_queue_size_)
        {
          history_queue_.pop_front();
        }
        last_state = state;
        return state;
      }
      //小陀螺
      else if(is_rotting)
      {//选择旋转角度小的那一块装甲板
        double min_angle = matched_armor.yaw;
        for(auto &new_target : same_id_armors)
        {
          if(new_target.yaw < min_angle)
          {
            matched_armor = new_target;
          }
        }
        history_queue_.clear();
#ifdef KALMAN
        Eigen::VectorXd measurement = Eigen::VectorXd::Zero(7);
        //小陀螺时保持当前速度
        measurement << matched_armor.position[0], matched_armor.position[1],matched_armor.position[2],
        last_state[3],last_state[4],last_state[5],matched_armor.yaw;
        kf_->initialize(measurement);
        Eigen::VectorXd state = kf_->predict(0);
#endif
#ifdef EXTENDED_KALMAN
        Eigen::VectorXd measurement = Eigen::VectorXd::Zero(8);
        measurement << matched_armor.position[0], matched_armor.position[1],matched_armor.position[2],
        last_state[3],last_state[4],last_state[5],matched_armor.yaw, last_state[7];
        ekf_->initialize(measurement);
        Eigen::VectorXd state = ekf_->predict(0);
#endif
        // pf_->initialize(measurement);
        // Eigen::VectorXd state = pf_->predict(0);
        history_queue_.push_back(Armor(state.head(3),matched_armor.stamp,matched_armor.id,state[6]));
        last_state = state;
        return state;
      }
      //有相同id的装甲板但不匹配
      else  
      {
        history_queue_.clear();
#ifdef KALMAN
        Eigen::VectorXd measurement = Eigen::VectorXd::Zero(7);
        measurement << matched_armor.position[0], matched_armor.position[1],matched_armor.position[2],
        last_state[3],last_state[4],last_state[5],matched_armor.yaw;
        kf_->initialize(measurement);
        Eigen::VectorXd state = kf_->predict(0);
#endif
#ifdef EXTENDED_KALMAN
        Eigen::VectorXd measurement = Eigen::VectorXd::Zero(8);
        measurement << matched_armor.position[0], matched_armor.position[1],matched_armor.position[2],
        last_state[3],last_state[4],last_state[5],matched_armor.yaw, last_state[7];
        ekf_->initialize(measurement);
        Eigen::VectorXd state = ekf_->predict(0);
#endif
        // pf_->initialize(measurement);
        // Eigen::VectorXd state = pf_->predict(0);
        history_queue_.push_back(Armor(state.head(3),matched_armor.stamp,matched_armor.id,state[6]));
        last_state = state;
        return state;
      }
    }
    else
    {
      lost_cnt_++;
      //连续丢失5帧进入lost
      if(lost_cnt_ > 5)
      {
        this->state_ = tracker::state::LOST;
        initialize(targets);
#ifdef KALMAN
        Eigen::VectorXd state = kf_->predict(0);
#endif
#ifdef EXTENDED_KALMAN
        Eigen::VectorXd state = ekf_->predict(0);
#endif
        // Eigen::VectorXd state = pf_->predict(0);

        return state;
      }
      else
      {
        double predict_t = lost_cnt_*dt;
#ifdef KALMAN
        Eigen::VectorXd state = kf_->predict(0);
#endif
#ifdef EXTENDED_KALMAN
        Eigen::VectorXd state = ekf_->predict(0);
#endif
        return state;
      }
    }// end else
  } //end else
} //end update()

} //namespace post_processor