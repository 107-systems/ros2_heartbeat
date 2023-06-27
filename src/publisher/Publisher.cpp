/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/ros2_heartbeat/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <ros2_heartbeat/publisher/Publisher.h>

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
  std::chrono::milliseconds const heartbeat_rate,
  std::chrono::milliseconds const heartbeat_deadline,
  std::chrono::milliseconds const heartbeat_liveliness_lease_duration)
: _heartbeat_start{std::chrono::steady_clock::now()}
, _heartbeat_qos_profile
{
  rclcpp::KeepLast(1),
  rmw_qos_profile_default
}
{
  _heartbeat_qos_profile.deadline(heartbeat_deadline);
  _heartbeat_qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
  _heartbeat_qos_profile.liveliness_lease_duration(heartbeat_liveliness_lease_duration);

  _heartbeat_pub = node_hdl.create_publisher<std_msgs::msg::UInt64>(
    heartbeat_topic,
    _heartbeat_qos_profile
    );

  _heartbeat_loop_timer = node_hdl.create_wall_timer(
    heartbeat_rate,
    [this]()
    {
      auto const node_up_time = std::chrono::steady_clock::now() - _heartbeat_start;

      std_msgs::msg::UInt64 heartbeat_msg;
      heartbeat_msg.data = std::chrono::duration_cast<std::chrono::seconds>(node_up_time).count();
      _heartbeat_pub->publish(heartbeat_msg);
    });
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* heartbeat */
