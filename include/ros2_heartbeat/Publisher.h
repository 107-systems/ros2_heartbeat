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

class Publisher
{
public:
  typedef std::shared_ptr<Publisher> SharedPtr;


  static SharedPtr create(rclcpp::Node & node_hdl, std::string const & heartbeat_topic, std::chrono::milliseconds const heartbeat_rate) {
    return std::make_shared<Publisher>(node_hdl, heartbeat_topic, heartbeat_rate);
  }


  Publisher(
    rclcpp::Node & node_hdl,
    std::string const & heartbeat_topic,
    std::chrono::milliseconds const heartbeat_rate
    );


private:
  std::chrono::steady_clock::time_point _heartbeat_start;

  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr _heartbeat_pub;
  rclcpp::TimerBase::SharedPtr _heartbeat_loop_timer;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* heartbeat */
