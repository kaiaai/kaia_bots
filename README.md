# Kaia.ai Arduino/ROS2 home robots

[Kaia.ai](https://kaia.ai) is a software platform for intelligent robots. Please read our blog for [more](https://kaia.ai/blog/what-is-kaia-ai/).

Please sign up for a Kickstarter pre-launch invite [here](https://remake.ai). We plan to launch on Kickstarter in 2nd half of 2024 via our [Maker's Pet](https://makerspet.com) hardware partner's campaign.

Please visit the [Support Forum](https://github.com/makerspet/support/discussions/)!

Kaia.ai robotics software platform is actively evolving and currently consists of these parts:
- Micro-ROS Arduino library for Kaia.ai-compatible robots [repo](https://github.com/kaiaai/micro_ros_arduino_kaia)
- End-user and development ROS2 Docker images [repo](https://github.com/kaiaai/install)
- Robot simulation ROS2 packages [repo](https://github.com/kaiaai/kaiaai_simulations)
- Robot operation ROS2 packages [repo](https://github.com/kaiaai/kaiaai), including SLAM mapping, navigation, etc.
- [WebRTC-based](https://github.com/kaiaai/kaiaai_python) image/video/data streaming
  - [Python-based](https://github.com/kaiaai/kaiaai_python) image/audio sensing, processing (ML), decision making (ML/AI), robot face animation (TODO)
- Cloud software infrastructure (TODO)
- robot skills store (TODO)

## Kaia.ai compatible robots
- Maker's Pet [Loki](https://github.com/makerspet/makerspet_loki) 200mm 3D-printable pet robot
- Maker's Pet [Fido](https://github.com/makerspet/makerspet_fido) 250mm 3D-printable pet robot
- Maker's Pet [Snoopy](https://github.com/makerspet/makerspet_snoopy) 300mm 3D-printable pet robot
- Maker's Pet Mini 120mm
- Add your own version to the [list](https://github.com/topics/kaiaai-robot)

## Setup
- View build, setup and bringup [videos](https://www.youtube.com/playlist?list=PLOSXKDW70aR8SA16wTB0ou9ClKhv7micy)
  - This video is somewhat outdated; an up-to-date video will be published at a later time

## Supported LiDAR/LDS sensors
- YDLIDAR X4 (default), X2/X2L, X3, X3PRO, SCL
- Neato XV11
- Xiaomi Roborock 1st gen LDS02RR (~$16 off AliExpress including shipping)
- SLAMTEC RPLIDAR A1
- 3irobotix Delta-2A, Delta-2B, Delta-2G
- LDROBOT LD14P
- Camsense X1

The entire up-to-date list of supported LiDAR is [here](https://github.com/kaiaai/LDS).

## Command cheat sheets

Example: operate [makerspet_loki](https://github.com/makerspet/makerspet_loki) robot using commands below.
Operate Fido or Snoopy by replacing `loki` with `fido` or `snoopy` in commands below.

### Launch Docker image to operate your robot

The Docker image contains ROS2 and micro-ROS pre-configured with additional Kaia.ai ROS2 packages.

Open a Windows command shell or Windows PowerShell window and type the command below. This should give you a bash prompt.
```
docker pull kaiaai/kaiaai:humble
docker run --name makerspet -it --rm -p 8888:8888/udp -p 4430:4430/tcp -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 kaiaai/kaiaai:humble
```

Get an aditional bash prompt by opening another Windows command shell or Windows PowerShell window and typing:
```
docker exec -it makerspet bash
```

### Operate a physical robot
```
# Launch the physical robot
ros2 launch kaiaai_bringup physical.launch.py robot_model:=makerspet_loki

# Drive robot manually
ros2 run kaiaai_teleop teleop_keyboard robot_model:=makerspet_loki

# Monitor robot sensors
ros2 launch kaiaai_bringup monitor_robot.launch.py robot_model:=makerspet_loki

# Create a map while driving manually
ros2 launch kaiaai_bringup cartographer.launch.py robot_model:=makerspet_loki

# Save the newly-created map
ros2 run nav2_map_server map_saver_cli -f ~/map --ros-args -p save_map_timeout:=60.0

# Robot self-drives using an existing map
ros2 launch kaiaai_bringup navigation.launch.py robot_model:=makerspet_loki map:=$HOME/map
```

Create a map automatically - no manual driving
```
# Launch the physical robot
ros2 launch kaiaai_bringup physical.launch.py robot_model:=makerspet_loki

# Launch SLAM (simultaneous localization and mapping) - navigate and map simultaneously
ros2 launch kaiaai_bringup navigation.launch.py robot_model:=makerspet_loki slam:=True

# Robot automatically seeks out, self-drives to unknown locations
ros2 launch explore_lite explore.launch.py

# Save the newly-created map
ros2 run nav2_map_server map_saver_cli -f ~/map --ros-args -p save_map_timeout:=60.0
```

### Specify LDS/LiDAR model to use
```
ros2 launch kaiaai_bringup physical.launch.py robot_model:=makerspet_loki lds_model:=XIAOMI-LDS02RR
ros2 launch kaiaai_bringup physical.launch.py robot_model:=makerspet_loki lds_model:=YDLIDAR-X4
ros2 launch kaiaai_bringup physical.launch.py robot_model:=makerspet_loki lds_model:=YDLIDAR-X3
ros2 launch kaiaai_bringup physical.launch.py robot_model:=makerspet_loki lds_model:=YDLIDAR-X3-PRO
ros2 launch kaiaai_bringup physical.launch.py robot_model:=makerspet_loki lds_model:=YDLIDAR-X2-X2L
ros2 launch kaiaai_bringup physical.launch.py robot_model:=makerspet_loki lds_model:=YDLIDAR-SCL
ros2 launch kaiaai_bringup physical.launch.py robot_model:=makerspet_loki lds_model:=NEATO-XV11
ros2 launch kaiaai_bringup physical.launch.py robot_model:=makerspet_loki lds_model:=3IROBOTIX-DELTA-2A
ros2 launch kaiaai_bringup physical.launch.py robot_model:=makerspet_loki lds_model:=3IROBOTIX-DELTA-2B
ros2 launch kaiaai_bringup physical.launch.py robot_model:=makerspet_loki lds_model:=3IROBOTIX-DELTA-2G
ros2 launch kaiaai_bringup physical.launch.py robot_model:=makerspet_loki lds_model:=LDROBOT-LD14P
ros2 launch kaiaai_bringup physical.launch.py robot_model:=makerspet_loki lds_model:=CAMSENSE-X1
ros2 launch kaiaai_bringup physical.launch.py robot_model:=makerspet_loki lds_model:=SLAMTEC-RPLIDAR-A1
```

### View, set physical robot's parameters
```
# View parameters
ros2 node list
ros2 node info /pet
ros2 param list /pet
ros2 param dump /pet

# Set the desired laser scan frequency to 7 Hz
ros2 param set /pet lds.scan_freq 7.0

# Get the current desired laser scan frequency
ros2 param get /pet lds.scan_freq

# Reset the desired laser scan frequency to default
ros2 param set /pet lds.scan_freq 0.0
```

### Operate a simulated robot

```
# Launch the robot in a simulation - drive manually
ros2 launch kaiaai_gazebo world.launch.py robot_model:=makerspet_loki
ros2 run kaiaai_teleop teleop_keyboard robot_model:=makerspet_loki
ros2 launch kaiaai_bringup monitor_robot.launch.py robot_model:=makerspet_loki

# Launch the robot in a simulation - robot self-drives around
ros2 launch kaiaai_gazebo world.launch.py robot_model:=makerspet_loki
ros2 launch kaiaai_gazebo self_drive_gazebo.launch.py robot_model:=makerspet_loki
ros2 launch kaiaai_bringup monitor_robot.launch.py robot_model:=makerspet_loki

# Launch the robot in a simulation - create, save a map; robot self-drives around
ros2 launch kaiaai_gazebo world.launch.py robot_model:=makerspet_loki
ros2 launch kaiaai_bringup cartographer.launch.py use_sim_time:=true robot_model:=makerspet_loki
ros2 launch kaiaai_gazebo self_drive_gazebo.launch.py robot_model:=makerspet_loki
ros2 run nav2_map_server map_saver_cli -f ~/living_room_map --ros-args -p save_map_timeout:=60.0

# Launch the robot in a simulation - let it navigate automatically using an existing map
ros2 launch kaiaai_gazebo world.launch.py robot_model:=makerspet_loki
ros2 launch kaiaai_bringup navigation.launch.py use_sim_time:=true robot_model:=makerspet_loki \
  map:=/ros_ws/src/kaiaai_simulations/kaiaai_gazebo/map/living_room.yaml

# Launch the robot in a simulation - navigate and create a map simultaneously; save the map
ros2 launch kaiaai_gazebo world.launch.py robot_model:=makerspet_loki
ros2 launch kaiaai_bringup navigation.launch.py use_sim_time:=true robot_model:=makerspet_loki slam:=True
ros2 run nav2_map_server map_saver_cli -f ~/map --ros-args -p save_map_timeout:=60.0

# Launch the robot in a simulation - navigate and create a map simultaneously
# Robot seeks out, self-drives to unknown locations to complete the mapping
ros2 launch kaiaai_gazebo world.launch.py robot_model:=makerspet_loki
ros2 launch kaiaai_bringup navigation.launch.py use_sim_time:=true robot_model:=makerspet_loki slam:=True
ros2 launch explore_lite explore.launch.py
ros2 run nav2_map_server map_saver_cli -f ~/map --ros-args -p save_map_timeout:=60.0

# Launch the robot in a simulation - navigate and create a map simultaneously
# Robot seeks out, self-drives to unknown locations to complete the mapping, saves map
ros2 run auto_mapper auto_mapper map_path:=/ros_ws/map.yaml
ros2 launch kaiaai_gazebo world.launch.py robot_model:=makerspet_loki
ros2 launch kaiaai_bringup navigation.launch.py use_sim_time:=true robot_model:=makerspet_loki slam:=True

# Launch the robot in a simulation - navigate and create a map simultaneously
# Robot seeks out, self-drives to unknown locations to complete the mapping
ros2 launch kaiaai_gazebo world.launch.py robot_model:=makerspet_loki
ros2 launch kaiaai_bringup navigation.launch.py use_sim_time:=true robot_model:=makerspet_loki slam:=True
ros2 run nav2_wfd explore
```

### Add your own modifications to an existing robot
```
# Inspect, edit robot's URDF model
ros2 launch kaiaai_bringup inspect_urdf.launch.py robot_model:=makerspet_loki
ros2 launch kaiaai_bringup edit_urdf.launch.py robot_model:=makerspet_loki

# Convert URDF robot model file into SDF Gazebo simulation model file
ros2 run kaiaai_gazebo urdf2sdf.sh /ros_ws/src/makerspet_loki
cd /ros_ws && colcon build --symlink-install --packages-select makerspet_loki
```

## Acknowledgements
Initial versions of packages in this repo are based on ROBOTIS
[Turtlebot3 code](https://github.com/ROBOTIS-GIT/turtlebot3)

## Release notes
v0.10.0 in debug
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
  - `ros2 launch kaiaai_bringup physical.launch.py robot_model:=makerspet_loki lds_model:=3IROBOTIX-DELTA-2A`
  - `ros2 launch kaiaai_bringup physical.launch.py robot_model:=makerspet_loki lds_model:=3IROBOTIX-DELTA-2G`

2/5/2024
- added LiDAR/LDS laser distance scan sensors support
  - YDLIDAR X3, X3-PRO
  - `ros2 launch kaiaai_bringup physical.launch.py robot_model:=makerspet_loki lds_model:=YDLIDAR-X3`
  - `ros2 launch kaiaai_bringup physical.launch.py robot_model:=makerspet_loki lds_model:=YDLIDAR-X3-PRO`
  - Neato XV11 `ros2 launch kaiaai_bringup physical.launch.py robot_model:=makerspet_loki lds_model:=NEATO-XV11`
  - RPLIDAR A1 `ros2 launch kaiaai_bringup physical.launch.py robot_model:=makerspet_loki lds_model:=SLAMTEC-RPLIDAR-A1`
- split kaiaai_telemetry config into default and custom
  - `kaiaai/kaiaai_telemetry/config/telem.yaml` is the default config
  - `makerspet_loki/config/telem.yaml` is the custom config for the Loki robot model
  - `makerspet_fido/config/telem.yaml` is the custom config for the Fido robot model
  - `makerspet_snoopy/config/telem.yaml` is the custom config for the Snoopy robot model

1/28/2024
- added YDLIDAR X2 support to kaiaai_telemetry 
  - `ros2 launch kaiaai_bringup physical.launch.py robot_model:=makerspet_loki lds_model:=YDLIDAR-X2-X2L`

1/21/2024
- kaiaai_telemetry now supports multiple LiDAR/LDS laser distance scan sensors
  - added Xiaomi Mi LDS02RR; default is YDLIDAR X4
  - `ros2 launch kaiaai_bringup physical.launch.py robot_model:=makerspet_loki lds_model:=XIAOMI-LDS02RR`
  - `ros2 launch kaiaai_bringup physical.launch.py robot_model:=makerspet_loki lds_model:=YDLIDAR-X4`

12/11/2024
- added fully automatic self-driving to map exploration (frontier exploration m-explore)
  - `ros2 launch explore_lite explore.launch.py`
  - I think this code needs debug

12/7/2024
- added Nav2 SLAM (as an alternative to Google Cartographer)
  - Nav2 SLAM enables driving autonomously to a set goal *while* mapping (and while the map is still incomplete)
- increase the map saver default timeout
  - fixed the map saving command timing out on my "slow" laptop
