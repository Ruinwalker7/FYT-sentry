#ifndef SERIAL__ROBOT_BASE_NODE_HPP_
#define SERIAL__ROBOT_BASE_NODE_HPP_

#include <thread>
#include <memory>
#include "ros/ros.h"
#include "serial/transporter_interface.hpp"
#include "serial/fixed_packet_tool.hpp"
#include "rm_interfaces/SerialSendMsg.h"
#include "rm_interfaces/SerialReceiveMsg.h"

namespace serial
{
class RobotBaseNode
{
public:
  RobotBaseNode();
public:
  void listen_loop();

private:
  void gimbal_cmd_cb(const rm_interfaces::SerialSendMsgConstPtr& msg);

private:
  ros::NodeHandle nh_;
  std::unique_ptr<std::thread> listen_thread_;
  TransporterInterface::SharedPtr transporter_;
  FixedPacketTool<16>::SharedPtr packert_tool_;

  rm_interfaces::SerialReceiveMsg msg_;

  ros::Subscriber sub_;
  ros::Publisher pub_;
  bool first_rec_;
  unsigned char enemy_color_;
};

}   // namespace serial

#endif  // SERIAL__ROBOT_BASE_NODE_HPP_
