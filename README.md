# ORB SLAM3 ROS2 Wrapper

## 1. Install

### Dependencies

```bash
    sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
    sudo apt update

    sudo apt-get install build-essential
    sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev

    sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libjasper-dev

    sudo apt-get install libglew-dev libboost-all-dev libssl-dev

    sudo apt install libeigen3-dev
```

### 1.1 Pangolin:

```bash
    git clone https://github.com/stevenlovegrove/Pangolin.git -b v0.9.0
    cd Pangolin
    mkdir build && cd build
    cmake ..
    make
    sudo make install
```

### 1.2 Build ORB SLAM3

#### Clone the repo:

```bash
    git clone https://github.com/thien94/ORB_SLAM3.git ORB_SLAM3
```

#### Build

```bash
    cd ORB_SLAM3
    chmod +x build.sh
    ./build.sh
```

### 1.3 Clone to existing ros2 workspace with cv_bridge 

#### Clone the repo:

```bash
    git clone https://github.com/ViktorDanilin/orb_slam_ros_wrapper.git
```

#### Building package

```bash
    cd ~/colcon_ws
    MAKEFLAGS="-j4" colcon build --allow-overriding cv_bridge
```
