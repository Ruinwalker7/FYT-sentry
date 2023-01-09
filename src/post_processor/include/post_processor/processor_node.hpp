// Copyright FYT 邹承甫 2022

#ifndef POST_PROCESSOR__PROCESSOR_NODE_HPP
#define POST_PROCESSOR__PROCESSOR_NODE_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/Time.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "rm_interfaces/Point2d.h"
#include "rm_interfaces/AllTargets.h"
#include "rm_interfaces/SerialReceiveMsg.h"
#include "rm_interfaces/SerialSendMsg.h"
#include "rm_interfaces/TrackInfo.h"

#include "pnp_solver.hpp"
#include "armor.hpp"
#include "tracker.hpp"
#include "predictor.hpp"
#include "../../utils/include/utils.hpp"

namespace post_processor{

using target_filter = message_filters::Subscriber<rm_interfaces::AllTargets>;
using serial_filter = message_filters::Subscriber<rm_interfaces::SerialReceiveMsg>;
using my_sync_polices = message_filters::sync_policies::ApproximateTime<rm_interfaces::AllTargets,rm_interfaces::SerialReceiveMsg>;
using time_synchronizer = message_filters::Synchronizer<my_sync_polices>;

class ProcessorNode
{
public:
  explicit ProcessorNode();

private:
  void syncCallback(
    const rm_interfaces::AllTargetsConstPtr &targets,
    const rm_interfaces::SerialReceiveMsgConstPtr &serial
  );

  bool getTransform(const float &gimbal_yaw, const float &gimbal_pitch);

  bool cameraToWorld(const Eigen::Vector3d &camera_pt, Eigen::Vector3d &world_pt);

  bool worldToCamera(const Eigen::Vector3d &world_pt, Eigen::Vector3d &camera_pt);

  void publishMsg(const double &goal_yaw, const double &goal_pitch, const double &distance);

  bool processTargets();

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Publisher info_pub_;
  ros::Publisher vis_pub_;

  std::shared_ptr<target_filter> targets_sub_;
  std::shared_ptr<serial_filter> serial_sub_;
  std::shared_ptr<time_synchronizer> time_synchronizer_;
  std::unique_ptr<PnPSolver> pnp_solver_;
  std::unique_ptr<Tracker> tracker_;
  std::unique_ptr<Predictor> predictor_;
  

  Eigen::Quaterniond quaternion_;
  Eigen::Vector3d translation_;

  visualization_msgs::Marker position_marker_;
  visualization_msgs::Marker predict_marker_;
  visualization_msgs::Marker velocity_marker_;
  visualization_msgs::Marker aim_shoot_marker_;

  ros::Time time_now_;
  double pitch_;
  double yaw_;
  bool debug_;
};



} //namespace post_processor

#endif