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
#include <functional>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/u_int64.hpp>

#include "../DefaultConfig.h"

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
  typedef std::function<void()> OnLivelinessLostFunc;
  typedef std::function<void()> OnLivelinessGainedFunc;
  typedef std::function<void(rclcpp::QOSDeadlineRequestedInfo & event)> OnDeadlineMissedFunc;


  static SharedPtr create(
    rclcpp::Node & node_hdl,
    std::string const & heartbeat_topic,
    std::chrono::milliseconds const heartbeat_deadline,
    std::chrono::milliseconds const heartbeat_liveliness_lease_duration,
    OnLivelinessLostFunc const on_liveliness_lost_func,
    OnLivelinessGainedFunc const on_liveliness_gained_func,
    OnDeadlineMissedFunc const on_deadline_missed_func)
  {
    return std::make_shared<Monitor>(node_hdl, heartbeat_topic, heartbeat_deadline, heartbeat_liveliness_lease_duration, on_liveliness_lost_func, on_liveliness_gained_func, on_deadline_missed_func);
  }
  static SharedPtr create(
    rclcpp::Node & node_hdl,
    std::string const & heartbeat_topic,
    OnLivelinessLostFunc const on_liveliness_lost_func,
    OnLivelinessGainedFunc const on_liveliness_gained_func,
    OnDeadlineMissedFunc const on_deadline_missed_func)
  {
    return create(node_hdl, heartbeat_topic, DEFAULT_DEADLINE, DEFAULT_LIVELINESS_LEASE_DURATION, on_liveliness_lost_func, on_liveliness_gained_func, on_deadline_missed_func);
  }


  Monitor(
    rclcpp::Node & node_hdl,
    std::string const & heartbeat_topic,
    std::chrono::milliseconds const heartbeat_deadline,
    std::chrono::milliseconds const heartbeat_liveliness_lease_duration,
    OnLivelinessLostFunc const on_liveliness_lost_func,
    OnLivelinessGainedFunc const on_liveliness_gained_func,
    OnDeadlineMissedFunc const on_deadline_missed_func
    );


private:
  rclcpp::QoS _heartbeat_qos_profile;
  rclcpp::SubscriptionOptions _heartbeat_sub_options;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr _heartbeat_sub;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* heartbeat */
