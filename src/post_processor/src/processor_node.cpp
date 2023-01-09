// Copyright FYT 邹承甫 2022

#include "post_processor/processor_node.hpp"

namespace post_processor{

ProcessorNode::ProcessorNode()
{
  //读一些常规参数
  nh_.param("/debug",debug_,false);
  std::vector<double> translation;
  nh_.getParam("/camera_to_gimbal", translation);
  utils::toEigenMatrix(translation_,translation);

  //构造tracker
  std::vector<double> Fvec;
  std::vector<double> Pvec;
  std::vector<double> Qvec;
  std::vector<double> Rvec;
  std::vector<double> Hvec;
  nh_.getParam("/post_processor/Q",Qvec);
  nh_.getParam("/post_processor/F",Fvec);
  nh_.getParam("/post_processor/R",Rvec);
  nh_.getParam("/post_processor/P",Pvec);
  nh_.getParam("/post_processor/H",Hvec);  
  tracker_ = std::make_unique<Tracker>(
    Qvec,
    Pvec,
    Fvec,
    Hvec,
    Rvec,
    debug_
  );

  //构造predictor
  double air_fraction;
  nh_.param("/post_processor/air_fraction",air_fraction,0.01);
  predictor_ = std::make_unique<Predictor>(air_fraction);

  //构造pnp_solver
  std::vector<float> camera_matrix;
  std::vector<float> dist_coeffs;
  while((!nh_.hasParam("/camera/camera_matrix")) || (!nh_.hasParam("/camera/dist_coeffs")));
  nh_.getParam("/camera/camera_matrix",camera_matrix);
  nh_.getParam("/camera/dist_coeffs",dist_coeffs);
  pnp_solver_ = std::make_unique<PnPSolver>(camera_matrix,dist_coeffs);

  //话题订阅
  targets_sub_ = std::make_shared<target_filter>(nh_, "/all_targets", 10);
  serial_sub_ = std::make_shared<serial_filter>(nh_, "/serial_receive", 10);

  //构造publisher和synchoronizer
  time_synchronizer_ = std::make_shared<time_synchronizer>(my_sync_polices(100), *targets_sub_, *serial_sub_);
  pub_ = nh_.advertise<rm_interfaces::SerialSendMsg>("/serial_sendmsg",1,true);
  info_pub_ = nh_.advertise<rm_interfaces::TrackInfo>("/track_info",1,true);

  //初始化可视化marker
  if(debug_)
  {
    vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>( "visualization_marker", 0 );
    position_marker_.ns = "position";
    position_marker_.type = visualization_msgs::Marker::SPHERE;
    position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.15;
    position_marker_.color.a = 1.0;
    position_marker_.color.g = 1.0;
    predict_marker_.ns = "predict";
    predict_marker_.type = visualization_msgs::Marker::SPHERE;
    predict_marker_.scale.x = predict_marker_.scale.y = predict_marker_.scale.z = 0.15;
    predict_marker_.color.a = 1.0;
    predict_marker_.color.r = 1.0;
    velocity_marker_.type = visualization_msgs::Marker::ARROW;
    velocity_marker_.ns = "velocity";
    velocity_marker_.scale.x = 0.03;
    velocity_marker_.scale.y = 0.05;
    velocity_marker_.color.a = 1.0;
    velocity_marker_.color.b = 1.0;
    aim_shoot_marker_.type = visualization_msgs::Marker::ARROW;
    aim_shoot_marker_.ns = "aim_shot";
    aim_shoot_marker_.scale.x = 0.03;
    aim_shoot_marker_.scale.y = 0.05;
    aim_shoot_marker_.color.a = 1.0;
    aim_shoot_marker_.color.g = 1.0;
    aim_shoot_marker_.color.r = 1.0;
  }

  //注册回调函数
  time_synchronizer_->registerCallback(boost::bind(&ProcessorNode::syncCallback, this, _1, _2));
} //end constructor


void ProcessorNode::syncCallback(
  const rm_interfaces::AllTargetsConstPtr &targets,
  const rm_interfaces::SerialReceiveMsgConstPtr &serial)
{
  if(debug_)
  {
    ROS_INFO("start post processor!\n");
  }

  //获取时间戳，弹速，设置相机光心坐标
  time_now_ = targets->header.stamp;
  double bullet_speed = serial->bullet_speed;
  pnp_solver_->setOpticalCenter({targets->offset.x, targets->offset.y});

  //获取坐标变换
  getTransform(serial->yaw, serial->pitch);
  //将每个装甲板中心点转为世界系
  std::vector<Armor> Armors;
  if(!targets->armors.empty())
  {
    for(auto armor :targets->armors)
    {
      //1.pnp解算出相机坐标
      std::vector<cv::Point2f> img_pts{
        {armor.tl.x, armor.tl.y},
        {armor.bl.x, armor.bl.y},
        {armor.br.x, armor.br.y},
        {armor.tr.x, armor.tr.y}
        };
      bool is_big = (armor.id == 1 || armor.id == 8);
      Eigen::VectorXd pose = pnp_solver_->solvePnP(img_pts, is_big);
      //2.转换为世界坐标
      Eigen::Vector3d target_camera;
      target_camera << pose[0], pose[1], pose[2];
      Eigen::Vector3d target_world;
      Eigen::Vector3d normal_vector_camera;
      normal_vector_camera << pose[3], pose[4], pose[5];
      Eigen::Vector3d normal_vector_world;
      if(!cameraToWorld(target_camera,target_world) || !cameraToWorld(normal_vector_camera-translation_, normal_vector_world))
      {
        ROS_WARN("FAILED while transform\n");
        publishMsg(0,0,-1);
        return;
      }
      double armor_angle = atan2(normal_vector_world(0),normal_vector_world(1))*180/3.14;
      //压入Armors
      Armor armor_transformed(target_world,time_now_,armor.id,armor_angle);
      Armors.emplace_back(armor_transformed);
    }
  }
  //开始跟踪
  Eigen::VectorXd target_state;
  target_state = tracker_->update(Armors);
  if(target_state.norm() == 0)
  {//没找到要瞄准的目标
    if(debug_)
    {
      position_marker_.action = visualization_msgs::Marker::DELETE;
      velocity_marker_.action = visualization_msgs::Marker::DELETE;
      predict_marker_.action = visualization_msgs::Marker::DELETE;
      visualization_msgs::MarkerArray marker_array;
      marker_array.markers.emplace_back(position_marker_);
      marker_array.markers.emplace_back(predict_marker_);
      marker_array.markers.emplace_back(velocity_marker_);
      marker_array.markers.emplace_back(aim_shoot_marker_);
      vis_pub_.publish(marker_array);
    }
    publishMsg(0,0,-1);
    return;
  }
  if(debug_)
  {
    ROS_INFO("tracking target={%f,%f,%f,%f,%f,%f,%f}\n",target_state[0], target_state[1],
    target_state[2],target_state[3],target_state[4],target_state[5],target_state[6]);

    position_marker_.action = visualization_msgs::Marker::ADD;
    position_marker_.pose.position.x = target_state[1]/1000.0;
    position_marker_.pose.position.y = -target_state[0]/1000.0;
    position_marker_.pose.position.z = target_state[2]/1000.0;
  }
  //目标预测
  predictor_->setBulletSpeed(bullet_speed);
  predictor_->setPitch(pitch_);
  float goal_pitch = predictor_->predict(target_state);
  if(debug_)
  {
    ROS_INFO("predict target={%f,%f,%f}\n",target_state[0], target_state[1],
    target_state[2]);
  }
  //求俯仰偏航
  float goal_yaw = (-atan2(target_state[0], target_state[1])-yaw_)*180.0f/CV_PI;
  float distance = sqrt(pow(target_state[0],2) + pow(target_state[1],2) + pow(target_state[2],2))/1000.0f;
  if(!predictor_->is_ok)
  {
    float xy_distance = sqrt(pow(target_state[0],2) + pow(target_state[1],2));
    goal_pitch = (atan2(target_state[2], xy_distance) - pitch_)*180.0f/CV_PI;
  }
  publishMsg(goal_yaw,goal_pitch,distance);
  rm_interfaces::TrackInfo track_info_msg;
  //可视化目标点
  if(debug_)
  {
    std_msgs::Header marker_header;
    marker_header.stamp = time_now_;
    marker_header.frame_id = "base_link";
    position_marker_.header = predict_marker_.header 
      = velocity_marker_.header = aim_shoot_marker_.header = marker_header;

    predict_marker_.action = visualization_msgs::Marker::ADD;
    predict_marker_.pose.position.x = target_state[1]/1000.0;
    predict_marker_.pose.position.y = -target_state[0]/1000.0;
    predict_marker_.pose.position.z = target_state[2]/1000.0;

    velocity_marker_.action = visualization_msgs::Marker::ADD;
    velocity_marker_.points.clear();
    velocity_marker_.points.emplace_back(position_marker_.pose.position);
    geometry_msgs::Point arrow_end = position_marker_.pose.position;
    arrow_end.x += target_state[4]/2000.0;
    arrow_end.y += -target_state[3]/2000.0;
    arrow_end.z += target_state[5]/2000.0;
    velocity_marker_.points.emplace_back(arrow_end);

    aim_shoot_marker_.action = visualization_msgs::Marker::ADD;
    aim_shoot_marker_.points.clear();
    aim_shoot_marker_.points.emplace_back(geometry_msgs::Point());
    Eigen::Vector3d aim_direction_camera(0,0,10);
    Eigen::Vector3d aim_direction_world;
    cameraToWorld(aim_direction_camera, aim_direction_world);
    geometry_msgs::Point aim_end;
    aim_end.x = aim_direction_world[1];
    aim_end.y = -aim_direction_world[0];
    aim_end.z = aim_direction_world[2];
    aim_shoot_marker_.points.emplace_back(aim_end);

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.emplace_back(position_marker_);
    marker_array.markers.emplace_back(predict_marker_);
    marker_array.markers.emplace_back(velocity_marker_);
    marker_array.markers.emplace_back(aim_shoot_marker_);
    vis_pub_.publish(marker_array);
  }
} //end_callback


bool ProcessorNode::getTransform(const float &gimbal_yaw, const float &gimbal_pitch)
{
  this->yaw_ = gimbal_yaw/180.0*CV_PI;
  this->pitch_ = gimbal_pitch/180.0*CV_PI;
  quaternion_ = Eigen::AngleAxisd(yaw_,Eigen::Vector3d::UnitZ())*
                Eigen::AngleAxisd(pitch_-CV_PI/2.0,Eigen::Vector3d::UnitX());
  return true;
}


bool ProcessorNode::cameraToWorld(const Eigen::Vector3d &camera_pt, Eigen::Vector3d &world_pt)
{
  try
  {
    world_pt = quaternion_*(camera_pt + translation_);
  }
  catch(const std::exception& e)
  {
    // std::cerr << e.what() << '\n';
    ROS_ERROR("failed while transform to world frame\n");
    return false;
  }
  return true;
}


bool ProcessorNode::worldToCamera(const Eigen::Vector3d &world_pt, Eigen::Vector3d &camera_pt)
{
  try
  {
    camera_pt = quaternion_.conjugate()*world_pt - translation_;
  }
  catch(const std::exception& e)
  {
    // std::cerr << e.what() << '\n';
    ROS_ERROR("failed while transform to world frame\n");
    return false;
  }
  return true;
}


void ProcessorNode::publishMsg(const double &goal_yaw, const double &goal_pitch, const double &distance)
{
  rm_interfaces::SerialSendMsg msg;
  msg.yaw = goal_yaw;
  msg.pitch = goal_pitch;
  msg.distance = distance;
  if(debug_){
      ROS_INFO("send msg {%f,%f,%f}\n",msg.yaw,msg.pitch,msg.distance);
  }
  pub_.publish(msg);
  rm_interfaces::TrackInfo track_info_msg;
  if(distance == -1)
  {
    track_info_msg.is_tracking = false;
  }
  else
  {
    track_info_msg.is_tracking = true;
    track_info_msg.RPY.z = fabs(goal_yaw) > 10 ? goal_yaw : 0;
  }
  info_pub_.publish(track_info_msg); 
}

}