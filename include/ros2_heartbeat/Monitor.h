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

#include <rclcpp/qos.hpp>
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


  static SharedPtr create(
    rclcpp::Node & node_hdl,
    std::string const & heartbeat_topic,
    std::chrono::milliseconds const heartbeat_deadline,
    std::chrono::milliseconds const heartbeat_liveliness_lease_duration)
  {
    return std::make_shared<Monitor>(node_hdl, heartbeat_topic, heartbeat_deadline, heartbeat_liveliness_lease_duration);
  }


  Monitor(
    rclcpp::Node & node_hdl,
    std::string const & heartbeat_topic,
    std::chrono::milliseconds const heartbeat_deadline,
    std::chrono::milliseconds const heartbeat_liveliness_lease_duration
    );


private:
  rclcpp::QoS _heartbeat_qos_profile;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr _heartbeat_sub;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* heartbeat */
