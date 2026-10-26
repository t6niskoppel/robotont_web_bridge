# robotont_web_bridge

![ROS 2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg) ![License](https://img.shields.io/badge/License-Apache_2.0-green.svg)

## Overview

ROS 2 services for the Robotont dashboard and Blockly. The frontend is a
separate repository, [robotont_webapp](https://github.com/robotont/robotont_webapp).

## Setup once

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

The Robotont laptop container uses `/ros2_ws`.

## Run each time

Source ROS and the workspace in the terminal that runs the bridge. Set the
**actual** robot domain and namespace; examples last checked on robotont-54:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROBOTONT_NAMESPACE=rt54
ros2 launch robotont_web_bridge web.launch.py namespace:=rt54 use_sim_time:=false
```

In the laptop container, source `/ros2_ws/install/setup.bash`. Leave this command
running, then start the webapp separately. Ctrl+C stops the bridge. The webapp
alone cannot discover robots or display sensors.

| Target | Domain | Namespace | `use_sim_time` |
|---|---|---|---|
| Robotont-54, last observed | `0` | `rt54` | `false` |
| Robotont-34, last observed | `34` | empty | `false` |
| Gazebo example | unused, e.g. `55` | match simulator, e.g. `rt5` | `true` |

Export the chosen domain before launching. A running process does not pick up
later exports. One bridge sees one ROS domain; robot selection does not cross
domains. Do not run Gazebo in a physical robot's domain.

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
use a trusted network. Terminal credentials protect ttyd only.

The `web_bridge` node publishes robot discovery and directional scan distances,
relays movement with a 0.5 s expiry, and controls the LED module. Its JSON API
uses `/web/robots`, `/web/distances`, `/web/command`, `/web/stop`, `/web/led`.
Robot topics remain in their own namespace, e.g. `/rt54/cmd_vel`.

## Cameras and troubleshooting

RGB discovery prefers `/camera/color/image_raw` or `/camera/image_raw` in the
robot namespace. If a `/compressed` publisher exists, the robot entry advertises
`rgb_transport: compressed`; otherwise it advertises `raw`, including for Gazebo.
The browser receives MJPEG from the video server in either case. Requests use sensor-data QoS so
best-effort camera publishers are compatible. Depth uses an
existing aligned, rectified or raw depth topic and retains raw ROS transport.

- **Page says disconnected:** start the bridge, not just the frontend; check
  that nothing else owns ports 9090/8080/7681.
- **Connected, no robot:** check domain/namespace and that a driver subscribes
  to the robot's `cmd_vel`. Check actual publishers, not only a cached topic list.
- **No camera:** a listed topic does not prove frames arrive. Check camera
  messages on the robot before blaming the browser. Robotont-54's camera
  configured a stream but produced no received raw/compressed frames in the
  2026-10-26 test; changing transport did not fix that physical-camera issue.
- **Package not found:** source the workspace in this terminal after building.

Tests (no ROS node or robot commands):

```bash
python3 -m unittest discover -s test -v
```

## License

[Apache 2.0](LICENSE).
