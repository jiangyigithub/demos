# load road side ros2 project setp by step

# step 0 install ros2 
https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Binary.html
https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

# step 1 start ros2

# step 2 ,install ros extensions packages

# step 3 part build gen5 msgs package
colcon build --packages-select radar_gen5_msgs

source install/radar_gen5_msgs/share/radar_gen5_msgs/local_setup.bash

# step 4 revise cmake file
else()

  set(CMAKE_CXX_STANDARD 14)


  set(FastRTPS_INCLUDE_DIR /opt/ros/$ENV{ROS_DISTRO}/include)
  set(FastRTPS_LIBRARY_RELEASE /opt/ros/$ENV{ROS_DISTRO}/lib/libfastrtps.so)

# step 5  build common and source bash
colcon build --packages-select radar_gen5_common
#path install/radar_gen5_common/share/radar_gen5_common
source install/radar_gen5_common/share/radar_gen5_common/local_setup.bash

# step 6  part build per kit msgs
colcon build --packages-select perception_kit_msgs

# step 7 build clock
colcon build --packages-select clock_publisher

# step 8 build all
colcon build

# step 9 run 
chmod 777 start_bt.sh
sudo ./start_bt.sh

# step 10 replay
#jan5szh@SZHPC12289:~/ros2_ws/bag/2021_03_18$
ros2 bag info rosbag2_2021_03_18-16_00_53
ros2 bag play rosbag2_2021_03_18-16_00_53

# submodule repo link
https://sourcecode.socialcoding.bosch.com/projects/IAD
https://sourcecode.socialcoding.bosch.com/projects/PERCEPTION_KIT
https://sourcecode.socialcoding.bosch.com/projects/CRAEV
https://sourcecode.socialcoding.bosch.com/projects/RS
