## Lidar-Inertial SLAM


## 1. Prerequisites
### 1.1. **Ubuntu 18.04** and **ROS Noetic**
The code may work in an newer version of ubuntu and ROS but not tested.

### 1.2 Livox-SDK Installation

1. Download or clone [Livox-SDK](https://github.com/Livox-SDK/Livox-SDK) from Github to local;

2. Refer to the corresponding [README.md](https://github.com/Livox-SDK/Livox-SDK/blob/master/README.md) document to install and run Livox-SDK;

## 2. Build
To build, run the following commands:
```
git clone https://github.com/Alexwei92/lidar_slam.git
cd lidar_slam
git submodule update --init
cd ../..
catkin_make
source devel/setup.bash
```
