# ROS2-E2E-Evaluation

This document will introduce how to use our ROS 2 end-to-end evaluation tool.  

ROS 2 version: Galactic.

# Setup:

1. Install LTTng:  
sudo apt-add-repository ppa:lttng/stable-2.12  
sudo apt-get update  
sudo apt-get install lttng-tools lttng-modules-dkms liblttng-ust-dev  
sudo apt-get install python3-babeltrace python3-lttng  

2. Install ROS 2 via Debian Packages  
https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html  

3. Build trace-related package  
sudo apt install -y \  
  ros-galactic-ros2trace \  
  ros-galactic-ros2trace-analysis \  
  ros-galactic-tracetools \  
  ros-galactic-tracetools-analysis \  
  ros-galactic-tracetools-launch \  
  ros-galactic-tracetools-read \  
  ros-galactic-tracetools-test \  
  ros-galactic-tracetools-trace

4. Build Autoware_Perf  
source /opt/ros/galactic/setup.bash  
mkdir -p ~/autoware_perf_galactic  
git clone -b galactic_add_tp https://gitlab.com/reishikou/ros2_tracing.git  
git clone -b galactic_add_tp https://gitlab.com/reishikou/tracetools_analysis.git  
git clone -b galactic_add_tp https://github.com/reishikou/rclcpp.git  
git clone -b galactic_add_tp https://github.com/reishikou/rcl.git  
colcon build  

# Run a Autoware demo:   

1. Build Autoware.Universe:  
git clone https://github.com/tier4/autoware.proj.gsm8.git -b tier4/universe autoware.proj  
cd autoware.proj  
mkdir src  
vcs import src < autoware.proj.repos  
vcs import src < missing_packages.repos  
touch src/vendor/grid_map/grid_map_costmap_2d/COLCON_IGNORE  
sudo apt install libnvidia-compute-470  
sudo apt install libnvidia-decode-470  
sudo apt install libnvidia-encode-470  
sudo apt install libnvidia-gl-470  
sudo apt install nvidia-kernel-common-470  
sudo apt install libnvidia-extra-470  
sudo apt install libnvidia-fbc1-470  
sudo apt install nvidia-driver-470  
sudo apt install libnvidia-ifr1-470  
sudo apt install nvidia-dkms-470  
./setup_ubuntu20.04.sh  
source /opt/ros/galactic/setup.bash   
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=off  --symlink-install --packages-skip-build-finished --continue-on-error --parallel-workers 3  

// Write it into bashrc.  
source /opt/ros/galactic/setup.bash   

2. Run and measure the application:  
terminal 1:  
source ~/autoware_perf_galactic/install/setup.bash  
ros2 trace  
terminal 2:  
sudo sysctl -w net.core.rmem_max=2147483647  
sudo ifconfig lo multicast  
ros2 launch autoware_launch logging_simulator.launch.xml vehicle_model:=gsm8 sensor_model:=aip_x2 map_path:=/[absolute path]/office_map/ rviz:=true perception:=false   
terminal 3:  
ros2 bag play /[absolute path]/rosbag/[rosbag file(.db3)] --topics /sensing/lidar/top/velodyne_packets /sensing/lidar/left/velodyne_packets /sensing/lidar/right/velodyne_packets /vehicle/status/twist /sensing/imu/imu_data /clock  
terminal 4:
. /[absolute path]/send_start_and_goal.sh  

3. Analyze the trace data  
mkdir -p ~/tracelog  
ros2 trace-analysis process /path/to/trace/directory > tracelog.txt  
babeltrace /path/to/trace/directory | grep callback_start > callback_start.txt  
babeltrace /path/to/trace/directory | grep callback_end > callback_end.txt  
// Write the path of tracelog into "/path/to/trace/directory"  
cd ~/autoware_perf_galactic/tracetools_analysis/tracetools_analysis/analysis/galactic  
jupyter notebook  
// Click the e2elatency.ipynb  

4. Enter the jupyter interface  
// Write the path of ~/tracelog into path_prefix = " "  
// Use rqt and cb function to find the path of end-to-end  
// Write the node name into node1, node2...  
// Write the topic name that the node subscribes into topic1, topic2...  
// Run the code  
