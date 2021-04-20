# ROS2-E2E-Evaluation

This document will introduce how to use our ROS 2 end-to-end evaluation tool.  

# Setup:
1. Install LTTng:  
apt-add-repository ppa:lttng/stable-2.12  
apt-get update  
apt-get install lttng-tools  
apt-get install lttng-modules-dkms  
apt-get install liblttng-ust-dev  
apt-get install python3-lttngust  

2. Install development tools and ROS tools:  
sudo apt update && sudo apt install -y \  
  build-essential \  
  cmake \  
  git \  
  libbullet-dev \  
  python3-colcon-common-extensions \  
  python3-flake8 \  
  python3-pip \  
  python3-pytest-cov \  
  python3-rosdep \  
  python3-setuptools \  
  python3-vcstool \  
  wget  
% install some pip packages needed for testing  
python3 -m pip install -U \  
  argcomplete \  
  flake8-blind-except \  
  flake8-builtins \  
  flake8-class-newline \  
  flake8-comprehensions \  
  flake8-deprecated \  
  flake8-docstrings \  
  flake8-import-order \  
  flake8-quotes \  
  pytest-repeat \  
  pytest-rerunfailures \  
  pytest  
% install Fast-RTPS dependencies  
sudo apt install --no-install-recommends -y \  
  libasio-dev \  
  libtinyxml2-dev  
% install Cyclone DDS dependencies  
sudo apt install --no-install-recommends -y \  
  libcunit1-dev  

3. Install Python module:  
$ pip install pandas fire  

4. Build ROS2 Foxy (Fork version):  
$ mkdir -p ~/ros2_foxy_fork/src  
$ cd ~/ros2_foxy_fork  
$ wget https://gist.githubusercontent.com/hsgwa/bf2ca762072fa87a86df3e13a0d8b2d5/raw/bebcf6675c84f233ab4a50531161316769ad0d17/ros2.repos  
$ vcs import src < ros2.repos  
$ colcon build --symlink-install  
% you can update the repository without rebuild:  
$ vcs pull src    

5. Install flamegraph.pl:  
$ cd ~/.local/bin  
$ wget https://raw.githubusercontent.com/brendangregg/FlameGraph/master/flamegraph.pl -O ~/.local/bin/flamegraph.pl && chmod +x $_  
% you can also delete it:  
$ rm ~/.local/bin/flamegraph.pl  

# Run a demo:   
1. Build application baesd on ROS 2:  
$ mkdir -p ~/ros2_ws/src  
$ cd ~/ros2_ws  
$ git clone --recursive https://github.com/hsgwa/e2e_demo.git ./src/e2e_demo  
$ source ~/ros2_foxy_fork/install/setup.bash % Build on ROS 2 Foxy Fork  
$ colcon build --symlink-install  

2. Run and measure the application:  
$ cd ~/ros2_ws  
$ . ./install/setup.bash  
$ ros2 launch e2e_demo clock_demo.launch.py % Stop it by Ctrl+C  
($ ros2 launch e2e_demo demo.launch.py % Run without ROS time)  
$ ls ~/.ros/tracing/  
e2e_demo  e2e_demo_clock % e2e_demo: trace data, e2e_demo_clock: trace data with clock  
$ babeltrace ~/.ros/tracing/e2e_demo/ | grep -e rclcpp_publish -e rclcpp_subscribe % Check if the trace points are inserted  

3. Generate a architecture file:  
$ ls ~/.ros/tracing/e2e_demo  
ust  
$ . ~/ros2_ws/install/setup.bash  
$ cat ./architecture.json  
{  
  "target_path": {},  
  "..." []  
}  
$ # trace_draw_node_graph [/path/to/architecture] [/path/to/png]  
$ trace_draw_node_graph ./architecture.json ./architecture.png  

4. Analyze the trace data:  
$ . ~/ros2_foxy_fork/install/setup.bash  
$ cd ~/ros2_ws/src/e2e_demo/e2e_demo/analysis/  
$ jupyter-lab  
% You can realize the following function in jupyter:  
e2e_latency.ipynb ：visualize the End-to-End latency  
node_latency.ipynb ：visualize the node latency  
comm_latency.ipynb：visualize the latency between nodes  
callback_duration.ipynb：visualize the callback execution time  
cui.ipynb：run the CUI tool  

# Conference url:  
The source node can be downloaded from the following repositories:   
sample : https://github.com/hsgwa/e2e_demo  

for vcs install: https://gist.github.com/hsgwa/bf2ca762072fa87a86df3e13a0d8b2d5  

rclcpp: https://github.com/hsgwa/rclcpp/tree/devel_e2e_measurement  

cyclonedds: https://github.com/hsgwa/cyclonedds/tree/devel_timestamp_for_ros2  

fastdds: https://github.com/hsgwa/Fast-DDS/tree/devel_timestamp_for_ros2  

tracetools_analysis: https://gitlab.com/HasegawaAtsushi/tracetools_analysis/-/tree/devel_e2e_measurement  

ros2_tracing: https://gitlab.com/HasegawaAtsushi/ros2_tracing/-/tree/devel_e2e_measurement  
