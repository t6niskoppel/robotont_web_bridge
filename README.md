# robotont_web_bridge

![ROS 2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg) ![License](https://img.shields.io/badge/License-Apache_2.0-green.svg)

## Overview

ROS 2 services for the Robotont dashboard and Blockly. The frontend is a
separate repository, [robotont_webapp](https://github.com/robotont/robotont_webapp).

## Setup

Requires ROS 2 Jazzy and `robotont_msgs` in the same sourced workspace.
Repository: [t6niskoppel/robotont_web_bridge](https://github.com/t6niskoppel/robotont_web_bridge).
Place this checkout in the workspace's `src/` directory, then build from its root:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
sudo apt install ttyd
colcon build --symlink-install --packages-up-to robotont_web_bridge
source install/setup.bash
```

## Run
Source ROS and the workspace in the terminal that runs the bridge. Set the robot domain and namespace; examples on robotont-54:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROBOTONT_NAMESPACE=rt54
ros2 launch robotont_web_bridge web.launch.py namespace:=rt54 use_sim_time:=false
```

## Services and launch arguments

| Service | Default port |
|---|---|
| rosbridge websocket | `9090` |
| web_video_server | `8080` |
| ttyd browser terminal | `7681` |

`web.launch.py` accepts `namespace` (defaults to `ROBOTONT_NAMESPACE`, or empty
for all discovered robots), `use_sim_time` (false), `rosbridge_port` (9090) and
`video_port` (8080). ttyd uses environment variables `TTYD_PORT` (7681) and
`TTYD_CREDENTIAL` (`user:password`, unset by default). Run one stack per set of
ports. Services listen on all interfaces without authentication by default;
use a trusted network. 

## License

[Apache 2.0](LICENSE).
