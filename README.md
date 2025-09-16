
### 1. Install Dependencies 

```bash
sudo apt-get install libeigen3-dev libboost-all-dev libceres-dev

```
### 2. Install Opencv

```bash
git clone https://github.com/opencv/opencv_contrib/
cd opencv_contrib
git checkout 4.8.0
cd ..
git clone https://github.com/opencv/opencv/
cd opencv
git checkout 4.8.0
mkdir opencv/build/
cd opencv/build/
cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules ..
make -j8
sudo make install
```

## 3. clone the Openvins Repo:

```bash 
mkdir -p ~/workspace/catkin_ws_ov/src/
cd ~/workspace/catkin_ws_ov/src/
git clone https://github.com/rpng/open_vins/
cd ..
colcon build #ROS2

echo "source ~/workspace/catkin_ws_ov/install/setup.sh" >> ~/.bashrc
source ~/.bashrc

```

### 4. Install ROS2 Humble

``` follow instructions for ros2 install ```

https://github.com/Strroke21/V-SLAM-with-RTABMAP-and-Realsense-D4XX

### 5. edit config file in given as per your sensors and topics

```bash 
    estimator_config.yaml #set use_stereo: false (if mono cam else set true for stereo)
    kalibr_imucam_chain.yaml # add your camera topic in this file 
    kalibr_imu_chain.yaml # add imu topic in this file 

```

### 6. Launch the Node: (before launching node make sure IMU noise parameters are correct)

```bash

#camera

ros2 launch realsense2_camera rs_launch.py   enable_depth:=true   enable_color:=true   enable_sync:=true   depth_module.depth_profile:=640,480,60   rgb_camera.color_profile:=640,480,60 enable_sync:=true enable_gyro:=true enable_accel:=true unite_imu_method:=2 gyro_fps:=200 accel_fps:=200
```

```bash

#openvins

ros2 run ov_msckf run_subscribe_msckf --ros-args -p config_path:=/home/deathstroke/workspace/catkin_ws_ov/src/open_vins/config/rs_d455/estimator_config.yaml -p verbosity:=DEBUG  -p try_zupt:=true

#replace config_path with your path 

```

```bash

#launch imu filter 
ros2 run imu_filter_madgwick imu_filter_madgwick_node   --ros-args   -r imu/data_raw:=/camera/camera/imu   -r imu/data:=/imu/data   -p use_mag:=false
```

```bash

#### 

#launch rtabmap for stable odometry
ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/camera/camera/color/image_raw \
    camera_info_topic:=/camera/camera/color/camera_info \
    odom_topic:=/odomimu \
    approx_sync:=true \
    depth:=false \
    visual_odometry:=false \
    imu_topic:=/imu/data

```

