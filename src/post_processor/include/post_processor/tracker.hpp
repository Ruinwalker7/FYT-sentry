// Copyright FYT 邹承甫 2022

#ifndef POST_PROCESSOR__TRACKER_HPP
#define POST_PROCESSOR__TRACKER_HPP

#include <memory>
#include <vector>
#include <deque>

#include <Eigen/Core>
#include <ros/ros.h>

#include "armor.hpp"
#include "filters/kalman_filter.hpp"
#include "filters/particle_filter.hpp"
#include "filters/extended_kalman_filter.hpp"
#include "../../utils/include/utils.hpp"

#define KALMAN
#ifndef KALMAN
// #define EXTENDED_KALMAN
#endif
// #define PARTICLE

namespace post_processor{

namespace tracker
{
  typedef enum{
    LOST,
    TRACKING
  }state;
}

class Tracker
{
public:
  explicit Tracker(
    const std::vector<double> &Qvec,
    const std::vector<double>& Pvec,
    const std::vector<double>& Fvec,
    const std::vector<double>& Hvec,
    const std::vector<double>& Rvec,
    const bool& is_debug
  );

public:
  Eigen::VectorXd update(const std::vector<Armor> &armors);
private:
  void initialize(const std::vector<Armor> &armors);

private:
#ifdef KALMAN
  std::unique_ptr<KalmanFilter> kf_;
#endif
#ifdef PARTICLE
  std::unique_ptr<ParticleFilter> pf_;
#endif
#ifdef EXTENDED_KALMAN
  std::unique_ptr<ExtendedKalmanFilter> ekf_;
#endif
  std::deque<Armor> history_queue_;
  ros::Time last_time;
  Eigen::VectorXd last_state;

  double max_match_thresh_;
  int state_;
  int tracking_id_;
  int lost_cnt_;
  int max_queue_size_;
  bool debug_;
};

} //namespace post_processor

#endif  //POST_PROCESSOR__TRACKER_HPP