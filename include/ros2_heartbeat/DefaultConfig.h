/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/ros2_heartbeat/graphs/contributors.
 */

#pragma once

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <chrono>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace heartbeat
{

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

static std::chrono::milliseconds constexpr DEFAULT_PERIOD{100};
static std::chrono::milliseconds constexpr DEFAULT_DEADLINE{2 * DEFAULT_PERIOD};
static std::chrono::milliseconds constexpr DEFAULT_LIVELINESS_LEASE_DURATION{1000};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* heartbeat */
