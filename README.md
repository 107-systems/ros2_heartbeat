<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `ros2_heartbeat`
==============================
[![Build Status](https://github.com/107-systems/ros2_heartbeat/actions/workflows/ros2.yml/badge.svg)](https://github.com/107-systems/ros2_heartbeat/actions/workflows/ros2.yml)
[![Spell Check status](https://github.com/107-systems/ros2_heartbeat/actions/workflows/spell-check.yml/badge.svg)](https://github.com/107-systems/ros2_heartbeat/actions/workflows/spell-check.yml)

This packages provides both heartbeat publishers and heartbeat monitors for monitoring system state in a multi-node ROS application.

<p align="center">
  <a href="https://github.com/107-systems/l3xz"><img src="https://raw.githubusercontent.com/107-systems/.github/main/logo/l3xz-logo-memento-mori-github.png" width="40%"></a>
</p>

#### How-to-build
```bash
cd $COLCON_WS/src
git clone https://github.com/107-systems/ros2_heartbeat
cd $COLCON_WS
source /opt/ros/humble/setup.bash
colcon build --packages-select ros2_heartbeat
```
