# Bebop Vicon Control
Package combining vicon_bridge, bebop_autonomy, and pid to control the Parrot Bebop quadcopter using Vicon motion capture system.
The quadcopter is controlled through keyboard node or by calling service functions through API server.
# Setup
This package has a few dependencies that first need to be installed. Start by downloading and installing vicon_bridge and bebop_autonomy from source (directions should be available at the links):

https://github.com/AutonomyLab/bebop_autonomy

https://github.com/ethz-asl/vicon_bridge

And then install the following ROS packages:

Teleop twist keyboard: `sudo apt-get install ros-<distro>-teleop-twist-keyboard`

PID package: `sudo apt-get install ros-<distro>-pid`

(Where `<distro>` is replaced with your ROS distribution like indigo, jade, kinetic...)

# Launch
1. Connect to Bebop Wifi AP
2. `roslaunch bebop_vicon_control bebop_control.py`
3. `roslaunch bebop_vicon_control bebop_pid_keyboard.py`

# Usage
The project is broken up into 2 different launch files for editing convience. This enables you to keep the Bebop driver WiFi connection (which can be frustrating to reconnect with frequently) while you make iterative edits to the PID gains, keyboard-commanded locations

The current keyboard commands included the following:
- Ctl-c and 0 both cause an emergency stop
- 1 has the Bebop takeoff
- 2 initiates the Bebop landing manuever 
- 3,4,6,7 are mapped to different x,y,z positions in VICON coordinates
- 5 uses the autonomous landing function to land on another VICON object
  

The bebop_pid_keyboard launch file also spins up an API server node that allows you to control the Bebop with any other python script that uses the Bebop client object in `bebop_api_client.py`.

