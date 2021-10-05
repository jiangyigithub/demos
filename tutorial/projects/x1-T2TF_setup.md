
Prerequisites:

    ROS foxy is installed.

1. Setup of the ROS2 workspace with simulation data
Creat a workspace with the necessary repositories
mkdir -p ~/track_to_track_fusion_ws/src && cd ~/track_to_track_fusion_ws/src
 
git clone ssh://git@sourcecode.socialcoding.bosch.com:7999/perception_kit/perception_kit_msgs.git --branch migration/ROS2
git clone ssh://git@sourcecode.socialcoding.bosch.com:7999/perception_kit/perception_kit_visu.git --branch fix_foxy_visualization
git clone ssh://git@sourcecode.socialcoding.bosch.com:7999/perception_kit/perception_kit_object_transform.git --branch fix-angle-in-sim
git clone ssh://git@sourcecode.socialcoding.bosch.com:7999/iad/top95_general.git
git clone ssh://git@sourcecode.socialcoding.bosch.com:7999/perception_kit/track_to_track_fusion.git # if you receive errors during build, use commit cbbd251e283 instead
 
cd ~/track_to_track_fusion_ws

- Hints:

    The branch "fix_foxy_visualization" for the perception_kit_visu repository is currently needed to get the visualization running → might be merged to branch "migration/ROS2"
    The branch "fix-angle-in-sim" for the perception_kit_object_transform is currently needed to have a correct transformation of the simulation data from sensor to fusion coordinate system → more discussions object representation and transformations needed in TOP95 project
    The branch "feature/IAD-1601-migrate-new-t2t-fusion-to-ros2" for the perception_kit_object_transform is currently needed to as the ROS2 variant is not yet merged to the master → might be merged to master branch in the future

2. Configuration the Track-to-Track Fusion

The following configurations can be done to adapt the fusion to different sensor setups:

- create a parameter yaml file (like ~/track_to_track_fusion_ws/src/track_to_track_fusion/config/ros2/top95_fusion_with_8_sensors.yaml) to define:
    sensor tracker inputs (sensor names, topic names, minimal existence probabilities, ...)
    operation frame of the T2TF
    output (topic name, minimal existence probabilities, ...)
- create launch file (like ~/track_to_track_fusion_ws/src/top95_general/launch/top95_fusion_with_8_sensors.py) to:
    start the T2TF with the configuration yaml file from above
    start and configure the object transformation nodelets for the transformation of sensor tracker inputs from sensor coordinate system in to the fusion coordinate system (i.e. the "operation frame" in the T2TF configuration yaml)

3. Build the ROS2 workspace
Build
cd ~/track_to_track_fusion_ws
colcon buildsource
source install/setup.bash

4. Run the Track-to-Track Fusion with a simulation bag file from the ASFINAG proving ground

Copy the folder
\\bosch.com\dfsrb\DfsDE\DIV\M\NE\95_TOP_95\240_Exchange\Measurements\SimulationRosbags\2021_03_16_fusion_8sensors_clockto your machine.

- Launch the Track-to-Track Fusion
ros2 launch top95_general top95_fusion_with_8_sensors.py

- Data Visualization with RViz
In a new terminal:
source ~/track_to_track_fusion_ws/install/setup.bash
rviz2 -d ~/track_to_track_fusion_ws/src/top95_general/rviz/fusion_4R4C.rviz

- Replay ros bag
ros2 bag info xxx
ros2 bag play xxx