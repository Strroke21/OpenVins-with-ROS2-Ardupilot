
### 1. Install Dependencies 

```bash
sudo apt-get install libeigen3-dev libboost-all-dev libceres-dev

```

## 2. clone the Openvins Repo:

```bash 
mkdir -p ~/workspace/catkin_ws_ov/src/
cd ~/workspace/catkin_ws_ov/src/
git clone https://github.com/rpng/open_vins/
cd ..
colcon build #ROS2

echo "source ~/workspace/catkin_ws_ov/install/setup.sh" >> ~/.bashrc
source ~/.bashrc

```

### 3. Install ROS2 Humble

``` follow instructions for ros2 install ```

https://github.com/Strroke21/V-SLAM-with-RTABMAP-and-Realsense-D4XX

### 4. edit config file in /open_vins/config/rs_d455/

```bash 
    estimator_config.yaml #set use_stereo: false (if mono cam else set true for stereo)
    kalibr_imucam_chain.yaml # add your camera topic in this file 
    kalibr_imu_chain.yaml # add imu topic in this file 

```

### 5. Launch the Node: 

```bash

ros2 run ov_msckf run_subscribe_msckf --ros-args -p config_path:=/home/deathstroke/workspace/catkin_ws_ov/src/open_vins/config/rs_d455/estimator_config.yaml -p verbosity:=DEBUG  -p try_zupt:=true

#replace config_path with your path 

```
