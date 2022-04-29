# [PL-VINS](https://github.com/cnqiangfu/PL-VINS) branch for Open 4.2.0
**[PL-VINS: Real-Time Monocular Visual-Inertial SLAM with Point and Line Features](https://arxiv.org/pdf/2009.07462.pdf)**
## Modifications:
1. all CMakeFiles.txt: set(CMAKE_CXX_FLAGS "-std=c++14")
2. #include <opencv2/imgproc/types_c.h>
   - camera_model/src/chessboard/Chessboard.cc
3. CV_AA = cv::LINE_AA, CV_GRAY2BGR = cv::COLOR_GRAY2BGR, CV_RGB2GRAY = cv::COLOR_RGB2GRAY
   - camera_model/src/intrinsic_calib.cc
   - camera_model/src/calib/CameraCalibration.cc
   - camera_model/src/chessboard/Chessboard.cc
   - feature_tracker/src/feature_tracker_node.cpp
   - pose_graph/src/ThirdParty/DVision/BRIEF.cpp
   - vins_estimator/src/loop_closure/ThirdParty/DVision/BRIEF.cpp
4. cv::CALIB_CB_ADAPTIVE_THRESH, cv::CALIB_CB_NORMALIZE_IMAGE, cv::CALIB_CB_FILTER_QUADS, cv::CALIB_CB_FAST_CHECK
   - camera_model/src/chessboard/Chessboard.cc:
5. cv::INTER_LINEAR
   - feature_tracker/src/feature_tracker.cpp
   - feature_tracker/src/linefeature_tracker.cpp
6. cv::WINDOW_NORMAL
   - feature_tracker/src/linefeature_tracker.cpp
7. cv::FONT_HERSHEY_SIMPLEX
   - pose_graph/src/keyframe.cpp
   - pose_graph/src/pose_graph.cpp
8. #include <opencv2/opencv.hpp>, #include <opencv2/highgui.hpp>
   - vins_estimator/src/loop_closure/TemplatedLoopDetector.h
   - vins_estimator/src/loop_closure/loop_closure.h
   - vins_estimator/src/loop_closure/demoDetector.h

## 1. Prerequisites
### Ubuntu 20.04.4-LTS
* Python 3.8.10
* OpenCV 4.2.0

### ROS1 installation
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
export ROS1_DISTRO=noetic # kinetic=16.04, melodic=18.04, noetic=20.04
sudo apt-get install ros-$ROS1_DISTRO-desktop-full
sudo apt-get install python3-catkin-tools python3-osrf-pycommon # ubuntu 20.04
sudo apt-get install libeigen3-dev libboost-all-dev libceres-dev
```
```
echo "alias source_ros1=\"source /opt/ros/$ROS1_DISTRO/setup.bash\"" >> ~/.bashrc
echo "alias source_devel=\"source devel/setup.bash\"" >> ~/.bashrc
source ~/.bashrc
```

### [Ceres Solver installation](http://ceres-solver.org/installation.html)
```
sudo apt-get install cmake 
sudo apt-get install libgoogle-glog-dev libgflags-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install libeigen3-dev
sudo apt-get install libsuitesparse-dev
```
```
wget http://ceres-solver.org/ceres-solver-2.1.0.tar.gz
tar zxf ceres-solver-2.1.0.tar.gz
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver-2.1.0
make -j4
make install
```
## 2. Build on ROS
```
sudo apt install metis
source_ros1
```
```
make -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/rkuo/PL-VINS.git
cd ../
catkin build
source devel/setup.bash
```

## 3. Run on EuRoC dataset
Download [EuRoC MAV Dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). 
We suggust you select difficult sequences to test.

run in the ~/catkin_plvins/
```
	roslaunch plvins_estimator plvins_show_linepoint.launch
	rosbag play YOUR_PATH_TO_DATASET/MH_05_difficult.bag
```
or 
```
roslaunch plvins_estimator euroc_fix_extrinsic.launch        #This launch runs without loop
```

Now you should be able to run PL-VINS in the ROS RViZ. 

**Note that**: if you want obtain motion trajectory and compare it to your method. Please modify the ouput paths: /PL-VINS/vins_estimator/src/visualization.cpp (trajectory without loop) and /PL-VINS/pose_graph/src/pose_graph.cpp (trajectory with loop). 

**Note that**:It is an interesting thing we find that different CPU maybe yield different result whether VINS-Mono or PL-VINS, maybe the reason of ROS mechanism. Therefore, we suggest you test or compare methods on your machine by yourself. 

