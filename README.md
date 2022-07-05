## Lidar-Inertial SLAM


## 1. Prerequisites
### 1.1. **Ubuntu 18.04** and **ROS Noetic**
The code may work in an newer version of ubuntu and ROS but not tested.

### 1.2. Livox-SDK Installation

1. Download or clone [Livox-SDK](https://github.com/Livox-SDK/Livox-SDK) from Github to local;

2. Refer to the corresponding [README.md](https://github.com/Livox-SDK/Livox-SDK/blob/master/README.md) document to install and run Livox-SDK;

### 1.3. ROS dependencies
```
sudo apt install ros-$(ROS_VERSION)-pcl-ros 
sudo apt install ros-$(ROS_VERSION)-velodyne
sudo apt install ros-$(ROS_VERSION)-usb-cam
```

## 2. Build
To build, run the following commands:
```
git clone https://github.com/Alexwei92/lidar_slam.git
cd lidar_slam/src
git submodule update --init --recursive
cd ..
catkin_make
source devel/setup.bash
```

## Known Issues

1. If `catkin_make` does not compile, please add following line to the **FAST_LIO** [CMakeLists.txt](src/FAST_LIO/CMakeLists.txt)
```
add_dependencies(fastlio_mapping fast_lio_generate_messages_cpp)
```

2. For some reason, the compilation will fail on TX2 with `gcc` and `g++`. [Solution] Please use `clang` to compile.
```
# install clang-9 and clang++-9
sudo apt update
sudo apt -y install clang-9 clang++-9

# build with clang
catkin_make -DCMAKE_C_COMPILER=/usr/bin/clang-9 -DCMAKE_CXX_COMPILER=/usr/bin/clang++-9
```
