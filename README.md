# Kaia.ai Arduino/ROS2 home robots

[Kaia.ai](https://kaia.ai) is a software platform for intelligent robots. Please read our blog for [more](https://kaia.ai/blog/what-is-kaia-ai/).

Please sign up for a Kickstarter pre-launch invite [here](https://remake.ai). We plan to launch on Kickstarter in 2nd half of 2024 via our [Maker's Pet](https://makerspet.com) hardware partner's campaign.

Please visit the [Support Forum](https://github.com/makerspet/support/discussions/)!

Kaia.ai robotics software platform is actively evolving and currently consists of these parts:
- Micro-ROS Arduino library for Kaia.ai-compatible robots [repo](https://github.com/kaiaai/micro_ros_arduino_kaia)
  - Micro-ROS LiDAR telemetry receiver [package](https://github.com/kaiaai/kaiaai_telemetry)
- End-user and development ROS2 Docker images [repo](https://github.com/kaiaai/install)
- Robot Gazebo simulation ROS2 [package](https://github.com/kaiaai/kaiaai_gazebo)
- Kaia.ai Python ROS2 software wrapper [package](https://github.com/kaiaai/kaiaai)
- Robot operation ROS2 [repo](https://github.com/kaiaai/kaiaai_bringup), including SLAM mapping, navigation, frontier exploration, etc.
- Robot keyboard teleoperation [package](https://github.com/kaiaai/kaiaai_teleop)
- [WebRTC-based](https://github.com/kaiaai/kaiaai_python) image/video/data streaming
  - [Python-based](https://github.com/kaiaai/kaiaai_python) image/audio sensing, processing (ML), decision making (ML/AI), robot face animation (TODO)
- Cloud software infrastructure (TODO)
- robot skills store (TODO)

## Kaia.ai compatible robots
- Maker's Pet [Loki](https://github.com/makerspet/makerspet_loki) 200mm 3D-printed pet robot
- Maker's Pet [Fido](https://github.com/makerspet/makerspet_fido) 250mm 3D-printed pet robot
- Maker's Pet [Snoopy](https://github.com/makerspet/makerspet_snoopy) 300mm 3D-printed pet robot
- Maker's Pet [Mini](https://github.com/makerspet/makerspet_mini) 125mm 3D-printed educational robot
- Add your own version to the [list](https://github.com/topics/kaiaai-robot)

## Supported LiDAR sensors
- YDLIDAR X4 (default), X2/X2L, X3, X3PRO, SCL
- Neato XV11
- Xiaomi Roborock 1st gen LDS02RR (~$16 off AliExpress including shipping)
- SLAMTEC RPLIDAR A1
- 3irobotix Delta-2A, Delta-2B, Delta-2G
- LDROBOT LD14P
- Camsense X1

The entire up-to-date list of supported LiDAR is [here](https://github.com/kaiaai/LDS).

## Installation Instructions
There are two ways to install ROS2 and Kaia.ai on your PC - using Docker (Windows, Linux) or Ubuntu 24.04 (physical PC of virtual machine).
Follow the installation [instructions here](https://github.com/kaiaai/install).

Watch build, setup and bringup [videos](https://www.youtube.com/playlist?list=PLOSXKDW70aR8SA16wTB0ou9ClKhv7micy)
- Note: these videos are outdated; updated videos will be published shortly for Maker's Pet Mini

### Launch ROS2/Kaia.ai (Docker only)
The [Kaia.ai Docker image](https://hub.docker.com/repository/docker/kaiaai/kaiaai/general) contains ROS2 and micro-ROS
pre-configured with additional Kaia.ai ROS2 packages.

Open a Windows command shell or Windows PowerShell window and type the command below. This should give you a bash prompt.
Note that your `c:\maps` will be mapped to `/root/maps` to store navigation maps. Feel free to change `c:\maps` path to a more suitable location, e.g. `c:\Users\MyUserName\maps`.
```
docker pull kaiaai/kaiaai:iron
docker run --name makerspet -it --rm c:\maps:/root/maps -p 8888:8888/udp -p 4430:4430/tcp -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 kaiaai/kaiaai:iron
```

Get an aditional bash prompt by opening another Windows command shell or Windows PowerShell window and typing:
```
docker exec -it makerspet bash
```

If you installed ROS2/Kaia.ai without Docker directly on a Ubuntu PC/VM, just boot your Ubuntu PC to a bash prompt.

# Command cheat sheets

`makerspet_mini` is the default robot model. When using another robot model, change the default model using Kaia.ai CLI.
For example, the command below sets the default robot model to `makerspet_loki`.
```
kaia config robot.model makerspet_loki
```

## Operate a physical robot

### Manual operation

```
# Launch the physical robot
ros2 launch kaiaai_bringup physical.launch.py

# Drive robot manually
ros2 run kaiaai_teleop teleop_keyboard

# Monitor robot sensors
ros2 launch kaiaai_bringup monitor_robot.launch.py

# Create a map while driving manually
ros2 launch kaiaai_bringup cartographer.launch.py

# Save the newly-created map
ros2 run nav2_map_server map_saver_cli -f ~/maps/map --ros-args -p save_map_timeout:=60.0
```

### Physical robot self-drives automatically
```
# Launch the physical robot
ros2 launch kaiaai_bringup physical.launch.py

# Specify target location;; robot self-drives using an existing map
ros2 launch kaiaai_bringup navigation.launch.py map:=$HOME/map

# Launch SLAM (simultaneous localization and mapping) - navigate and map simultaneously
ros2 launch kaiaai_bringup navigation.launch.py slam:=True

# Robot automatically seeks out, self-drives to unknown locations
ros2 launch explore_lite explore.launch.py

# Save the newly-created map
ros2 run nav2_map_server map_saver_cli -f ~/maps/map --ros-args -p save_map_timeout:=60.0
```

## Use Command Line with Physical Robot

### Get, Set Robot's Parameters
```
# View parameters
ros2 node list
ros2 node info /pet
ros2 param list /pet
ros2 param dump /pet

# Get the current laser scan frequency
ros2 param get /pet lidar.scan.freq.now

# Set the desired laser scan frequency to 7 Hz
ros2 param set /pet lidar.scan.freq.target 7.0

# Get the current desired laser scan frequency
ros2 param get /pet lidar.scan.freq.target

# Reset the desired laser scan frequency to default
ros2 param set /pet lidar.scan.freq.target 0.0
```

### Monitor Robot's Telemetry
```
# List available topics
ros2 topic list

# Get WiFi strength
ros2 topic echo /wifi_state --once

# View raw telemetry
ros2 topic echo /telemetry --once

# Get LiDAR scan data
ros2 topic echo /scan --once

# View current odometer value
ros2 topic echo /odom --once

# View current wheel rotation angles
ros2 topic echo /joint_states --once

# View current battery voltage, charge percentage
ros2 topic echo /battery_state --once

# View target velocity sent by kaiaai_telem or navigation
ros2 topic echo /cmd_vel --once
```

## Operate robot in simulated environment

### Operate a simulated robot manually

```
# Launch the robot in a simulation - drive manually
ros2 launch kaiaai_gazebo world.launch.py
ros2 run kaiaai_teleop teleop_keyboard
ros2 launch kaiaai_bringup monitor_robot.launch.py

# Launch the robot in a simulation - robot self-drives around
ros2 launch kaiaai_gazebo world.launch.py
ros2 launch kaiaai_gazebo self_drive_gazebo.launch.py
ros2 launch kaiaai_bringup monitor_robot.launch.py

# Launch the robot in a simulation - let it navigate automatically using an existing map
ros2 launch kaiaai_gazebo world.launch.py
ros2 launch kaiaai_bringup navigation.launch.py use_sim_time:=true \
  map:=/ros_ws/src/kaiaai_gazebo/map/living_room.yaml

# Launch the robot in a simulation - navigate and create a map simultaneously; save the map
ros2 launch kaiaai_gazebo world.launch.py robot_model:=makerspet_loki
ros2 launch kaiaai_bringup navigation.launch.py use_sim_time:=true slam:=True
ros2 run nav2_map_server map_saver_cli -f ~/maps/map --ros-args -p save_map_timeout:=60.0
```

### Let robot self-drive autonomously

```
# Launch the robot in a simulation - create, save a map; robot self-drives around trivially
ros2 launch kaiaai_gazebo world.launch.py
ros2 launch kaiaai_bringup cartographer.launch.py use_sim_time:=true
ros2 launch kaiaai_gazebo self_drive_gazebo.launch.py
ros2 run nav2_map_server map_saver_cli -f ~/maps/living_room_map --ros-args -p save_map_timeout:=60.0

# Launch the robot in a simulation - navigate and create a map simultaneously
# Robot seeks out, self-drives to unknown locations to complete the mapping
ros2 launch kaiaai_gazebo world.launch.py
ros2 launch kaiaai_bringup navigation.launch.py use_sim_time:=true slam:=True
ros2 launch explore_lite explore.launch.py
ros2 run nav2_map_server map_saver_cli -f ~/maps/map --ros-args -p save_map_timeout:=60.0

# Launch the robot in a simulation - navigate and create a map simultaneously
# Robot seeks out, self-drives to unknown locations to complete the mapping, saves map
ros2 run auto_mapper auto_mapper map_path:=/ros_ws/map.yaml
ros2 launch kaiaai_gazebo world.launch.py
ros2 launch kaiaai_bringup navigation.launch.py use_sim_time:=true slam:=True

# Launch the robot in a simulation - navigate and create a map simultaneously
# Robot seeks out, self-drives to unknown locations to complete the mapping
ros2 launch kaiaai_gazebo world.launch.py
ros2 launch kaiaai_bringup navigation.launch.py use_sim_time:=true slam:=True
ros2 run nav2_wfd explore
```

## Advanced Configuration

### List of Supported LiDARs
You can set `lidar_model` to any of these supported models: `XIAOMI-LDS02RR`, `YDLIDAR-X4`, `YDLIDAR-X3`,
`YDLIDAR-X3-PRO`, `YDLIDAR-X2-X2L`, `YDLIDAR-SCL`, `NEATO-XV11`, `3IROBOTIX-DELTA-2A`, `3IROBOTIX-DELTA-2B`,
`3IROBOTIX-DELTA-2G`, `LDROBOT-LD14P`, `CAMSENSE-X1`, `SLAMTEC-RPLIDAR-A1`.


### Add your own modifications to an existing robot
```
# Inspect, edit robot's URDF model
ros2 launch kaiaai_bringup inspect_urdf.launch.py
ros2 launch kaiaai_bringup edit_urdf.launch.py

# Convert URDF robot model file into SDF Gazebo simulation model file
ros2 run kaiaai_gazebo urdf2sdf.sh /ros_ws/src/makerspet_loki
cd /ros_ws && colcon build --symlink-install --packages-select makerspet_loki
```

### Overriding default robot and LiDAR models per launch
Commands `physical.launch.py`, `teleop_keyboard`, `monitor_robot.launch.py`, `cartographer.launch.py`,
`navigation.launch.py`, `inspect_urdf.launch.py`, `edit_urdf.launch.py` and `world.launch.py` accept
an optional `robot_model` argument to override the default robot package setting, per launch.

The `physical.launch.py` also accepts an optional `lidar_model` argument to override the default
LiDAR model choice.

For example, the command below will use the `makerspet_loki` robot model and `YDLIDAR-X3` LiDAR instead of the defaults.
```
ros2 launch kaiaai_bringup physical.launch.py robot_model:=makerspet_loki lidar_model:=YDLIDAR-X3
```

## Acknowledgements
Initial versions of packages in this repo are based on ROBOTIS
[Turtlebot3 code](https://github.com/ROBOTIS-GIT/turtlebot3)

## Release notes
v0.10.0 in debug
- converted kaiaa from metapackage to Pyhon package
- kaia CLI sets default robot model
- added YDLIDAR SCL
  - added intensity telemetry publication
- added Maker's Pet Mini
  - added LiDAR orientation_deg

v0.9.0
- added WebRTC, web server, OpenCV launch
- added Camsense X1 LiDAR

v0.8.0
- added kaiaai_python ROS2 package
- added 3irobotix Delta-2A 230400 baud (vs 115200)
- added 3irobotix Delta-2B
- bugfix missing first scan point

v0.7.0
- kaiaai_telemetry
  - switched to KaiaTelemetry2 message
  - publish /battery_state
  - publish /wifi_state RSSI
  - discard_broken_scans, needs more debug
- added LDROBOT LD14P

v0.6.0 2/11/2024
- added 3irobotix Delta-2A, Delta-2G

2/5/2024
- added LiDAR laser distance scan sensors support
  - YDLIDAR X3, X3-PRO
  - Neato XV11
  - RPLIDAR A1
- split kaiaai_telemetry config into default and custom
  - `kaiaai/kaiaai_telemetry/config/telem.yaml` is the default config
  - `makerspet_loki/config/telem.yaml` is the custom config for the Loki robot model
  - `makerspet_fido/config/telem.yaml` is the custom config for the Fido robot model
  - `makerspet_snoopy/config/telem.yaml` is the custom config for the Snoopy robot model

1/28/2024
- added YDLIDAR X2 support to kaiaai_telemetry 

1/21/2024
- kaiaai_telemetry now supports multiple LiDAR laser distance scan sensors
  - added Xiaomi Mi LDS02RR; default is YDLIDAR X4

12/11/2024
- added fully automatic self-driving to map exploration (frontier exploration m-explore)
  - I think this code needs debug

12/7/2024
- added Nav2 SLAM (as an alternative to Google Cartographer)
  - Nav2 SLAM enables driving autonomously to a set goal *while* mapping (and while the map is still incomplete)
- increase the map saver default timeout
  - fixed the map saving command timing out on my "slow" laptop
