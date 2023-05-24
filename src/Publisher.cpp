/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/ros2_heartbeat/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <ros2_heartbeat/Publisher.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace heartbeat
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Publisher::Publisher(
  rclcpp::Node & node_hdl,
  std::string const & heartbeat_topic,
  std::chrono::milliseconds const heartbeat_rate)
: _heartbeat_start{std::chrono::steady_clock::now()}
{
  _heartbeat_pub = node_hdl.create_publisher<std_msgs::msg::UInt64>(heartbeat_topic, 1);

  _heartbeat_loop_timer = node_hdl.create_wall_timer(
    heartbeat_rate,
    [this]()
    {
      std_msgs::msg::UInt64 heartbeat_msg;

      heartbeat_msg.data = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - _heartbeat_start).count();

      _heartbeat_pub->publish(heartbeat_msg);
    });
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* heartbeat */
