/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/ros2_heartbeat/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <ros2_heartbeat/Monitor.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace heartbeat
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Monitor::Monitor(
  rclcpp::Node & node_hdl,
  std::string const & heartbeat_topic,
  std::chrono::milliseconds const heartbeat_timeout)
: _heartbeat_timeout{heartbeat_timeout}
, _prev_heartbeat_timepoint{std::chrono::steady_clock::now()}
{
  _heartbeat_sub = node_hdl.create_subscription<std_msgs::msg::UInt64>(
    heartbeat_topic,
    1,
    [this](std_msgs::msg::UInt64::SharedPtr const /* msg */)
    {
      _prev_heartbeat_timepoint = std::chrono::steady_clock::now();
    });

  RCLCPP_INFO(node_hdl.get_logger(), "Monitor subscribed to \"%s\".", heartbeat_topic.c_str());
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

std::tuple<bool, std::chrono::milliseconds> Monitor::isTimeout() const
{
  auto const now = std::chrono::steady_clock::now();
  auto const time_since_last_heartbeat = std::chrono::duration_cast<std::chrono::milliseconds>(now - _prev_heartbeat_timepoint);

  bool const is_timeout = (time_since_last_heartbeat > _heartbeat_timeout);
  return std::make_tuple(is_timeout, time_since_last_heartbeat);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* heartbeat */
