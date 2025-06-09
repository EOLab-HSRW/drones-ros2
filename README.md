# `eolab-ros2`

## Install

```
sudo pip install -U vcstool git+https://github.com/EOLab-HSRW/drones-fw.git
mkdir -p ~/eolab_ws/src
git clone https://github.com/EOLab-HSRW/drones-ros2.git && cd drones-ros2
vcs import < .repos
eolab_drones build --type sitl --drone protoflyer --msgs-output ./px4_msgs
cd ~/eolab_ws
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install
source install/setup.bash
```
