/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/ros2_heartbeat/graphs/contributors.
 */

#pragma once

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <tuple>
#include <chrono>
#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/u_int64.hpp>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace heartbeat
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Monitor
{
public:
  typedef std::shared_ptr<Monitor> SharedPtr;


  static SharedPtr create(rclcpp::Node & node_hdl, std::string const & heartbeat_topic, std::chrono::milliseconds const heartbeat_timeout) {
    return std::make_shared<Monitor>(node_hdl, heartbeat_topic, heartbeat_timeout);
  }


  Monitor(
    rclcpp::Node & node_hdl,
    std::string const & heartbeat_topic,
    std::chrono::milliseconds const heartbeat_timeout
    );


  std::tuple<bool, std::chrono::milliseconds> isTimeout() const;


private:
  std::chrono::milliseconds const _heartbeat_timeout;
  std::chrono::steady_clock::time_point _prev_heartbeat_timepoint;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr _heartbeat_sub;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* heartbeat */
