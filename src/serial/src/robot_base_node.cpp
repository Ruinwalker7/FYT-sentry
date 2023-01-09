#include "serial/robot_base_node.hpp"

#include <thread>
#include <memory>
#include <string>

#include "serial/uart_transporter.hpp"

namespace serial
{

RobotBaseNode::RobotBaseNode()
: nh_(), first_rec_(true), enemy_color_(1)
{
  std::string port_name;
  nh_.param("serial/port_name",port_name,std::string("/dev/ttyUSB0"));
  auto transporter = std::make_shared<serial::UartTransporter>(port_name);
  packert_tool_ = std::make_shared<FixedPacketTool<16>>(transporter);

  sub_ = nh_.subscribe<rm_interfaces::SerialSendMsg>(
    "serial_sendmsg", 100,
    &RobotBaseNode::gimbal_cmd_cb,this);
    
  pub_ = nh_.advertise<rm_interfaces::SerialReceiveMsg>(
    "serial_receive",100);
  
  listen_thread_ = std::make_unique<std::thread>(&RobotBaseNode::listen_loop,this);
}

void RobotBaseNode::gimbal_cmd_cb(const rm_interfaces::SerialSendMsgConstPtr& msg)
{
  FixedPacket<16> packet;
  packet.load_data<unsigned char>(0x00,1);
  packet.load_data<float>(msg->pitch,2);
  packet.load_data<float>(msg->yaw,6);
  packet.load_data<float>(msg->distance,10);
  //packet.set_check_byte();

  packert_tool_->send_packet(packet);
}

void RobotBaseNode::listen_loop()
{
  ros::Rate frequency(200);
  FixedPacket<16> packet;
  while (ros::ok())
  {
    if(packert_tool_->recv_packet(packet))
    {
      packet.unload_data(msg_.enemy_color,1);
      packet.unload_data(msg_.bullet_speed,2);
      packet.unload_data(msg_.pitch,6);
      packet.unload_data(msg_.yaw,10);

      if(first_rec_ || enemy_color_ != msg_.enemy_color)
      {
        enemy_color_ = msg_.enemy_color;
        nh_.setParam("/enemy_color", msg_.enemy_color);
        first_rec_ = false;
      }
    }
    if(!first_rec_)
    {
      std_msgs::Header hd;
      hd.stamp = ros::Time::now();
      msg_.header = hd;
      pub_.publish(msg_);
    }
    frequency.sleep();
  }
  
}

}   // namespace serial
