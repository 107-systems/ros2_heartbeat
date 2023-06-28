/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/ros2_heartbeat/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <ros2_heartbeat/monitor/Monitor.h>

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
  std::chrono::milliseconds const heartbeat_deadline,
  std::chrono::milliseconds const heartbeat_liveliness_lease_duration,
  OnLivelinessLostFunc const on_liveliness_lost_func,
  OnLivelinessGainedFunc const on_liveliness_gained_func,
  OnDeadlineMissedFunc const on_deadline_missed_func)
: _heartbeat_qos_profile
{
  rclcpp::KeepLast(1),
  rmw_qos_profile_default
}
{
  _heartbeat_qos_profile.deadline(heartbeat_deadline);
  _heartbeat_qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
  _heartbeat_qos_profile.liveliness_lease_duration(heartbeat_liveliness_lease_duration);

  _heartbeat_sub_options.event_callbacks.deadline_callback =
    [on_deadline_missed_func](rclcpp::QOSDeadlineRequestedInfo & event) -> void
    {
      if (on_deadline_missed_func)
        on_deadline_missed_func(event);
    };

  _heartbeat_sub_options.event_callbacks.liveliness_callback =
    [this, &node_hdl, on_liveliness_lost_func, on_liveliness_gained_func, heartbeat_topic](rclcpp::QOSLivelinessChangedInfo & event) -> void
    {
      if (event.alive_count == 0)
      {
        if (on_liveliness_lost_func)
          on_liveliness_lost_func();
      }
      else if (event.alive_count == 1)
      {
        if (on_liveliness_gained_func)
          on_liveliness_gained_func();
      }
      else
        RCLCPP_ERROR(node_hdl.get_logger(), "more than one publisher missed for \"%s\".", heartbeat_topic.c_str());
    };

  _heartbeat_sub = node_hdl.create_subscription<std_msgs::msg::UInt64>(
    heartbeat_topic,
    _heartbeat_qos_profile,
    [this](std_msgs::msg::UInt64::SharedPtr const /* msg */) { },
    _heartbeat_sub_options);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* heartbeat */
