# WebVR-Teleop
- Web platform to process VR controller data and send out to a robot arm
- Uses Aframe XR, ROS, and Mujoco

# Usage:
## Start ROSBridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

## Start Web Server (for now)
run web_files/server.py

## Run IK Reference Generator
from teleop_ros_ws: source install/setup.bash
ros2 run controller ik_ref_generator

## To test reference angles (if needed)
ros2 run simulator ik_sim 

## Go to Webpage
https://192.168.1.164:3000

## Set arm initial position if not outputted by a robot or simulator
ros2 topic pub /endeff_pos geometry_msgs/msg/Point "{x: 0.0, y: 0.45, z: 0.45}"

## Interaction
Put hand at end effector position and press A to begin tracking
Press B to stop tracking

# TODO
- General code cleanup
- Change handedness and support dual arm
- Add angular position and velocity logging
- Make a big launch file for whole project
- Use a more lightweight library for IK
- Considate parameters into one easy-for-user file

