0. ROS2 installation and extension:
- install ROS foxy .
https://docs.ros.org/en/foxy/Installation/Linux-Install-Debians.html
  1. start by source command
    source /opt/ros/foxy/local_setup.bash
  2. set ros2 as default on linux
    echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
  3. check default system environment
    gedit ~/.bashrc
  4. environment vairable
    printenv | grep -i ROS
- extension packages
    sudo apt update
    sudo apt install python3-colcon-common-extensions
    sudo apt install libyaml-cpp-dev 
    sudo apt install ~nros-foxy-rqt*
    sudo apt install ros-foxy-diagnostic-updater
    sudo apt-get install python-caktin-tools
    sudo apt-get install ros-foxy-ros2bag \
                        ros-foxy-rosbag2-converter-default-plugins \
                        ros-foxy-rosbag2-storage-default-plugins

1. ROS2 hello world
- Introducing turtlesim and rqt
https://docs.ros.org/en/foxy/Tutorials/Turtlesim/Introducing-Turtlesim.html

2. ROS2 concept and feature
```md
# ros2 system concept
## node --> minumum application unit
node
point to point
1 to multi

## communication model --> exchange nodes message
- 1. publisher-subscriber model
topic(单向)
publiser
subscribtion
msg

- 2. call-and-response model,
service(双向)
client
server
srv(request-response)

## nodes communication mechanism
DDS (middleware)
intra_process (config node option, set "use_intra_process_comms")

## integrate nodes as execuable, relationship with middleware and opteration system
executor
single/multi process
single/multi thread
```
(key words: Publisher/Subscriber/Topic//Nodes/Services/Quality of Service/ROS Client Library --> rclcpp)
- ROS2 concept
  - https://docs.ros.org/en/foxy/Concepts.html#internal-ros-interfaces
  - https://docs.ros.org/en/foxy/Concepts/About-Internal-Interfaces.html#internal-api-architecture-overview
- Understanding ROS 2 nodes
https://docs.ros.org/en/foxy/Tutorials/Understanding-ROS2-Nodes.html

3. package
(key words: CMakeLists.txt, package.xml)
Creating your first ROS 2 package
https://docs.ros.org/en/foxy/Tutorials/Creating-Your-First-ROS2-Package.html

4. build(build, install, log)
- Using colcon to build packages
https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html

```bash
colcon list
colcon graph
rm -rf install log build
```

5. source
When colcon has completed building successfully, the output will be in the install directory. Before you can use any of the installed executables or libraries, you will need to add them to your path and library paths. colcon will have generated bash/bat files in the install directory to help setup the environment. These files will add all of the required elements to your path and library paths as well as provide any bash or shell commands exported by packages.
. install/setup.bash

6. run/launch
Launch files allow you to start up and configure a number of executables containing ROS 2 nodes simultaneously.
Creating a launch file
https://docs.ros.org/en/foxy/Tutorials/Launch-Files/Creating-Launch-Files.html
https://docs.ros.org/en/foxy/How-To-Guides/Launch-file-different-formats.html
```bash
# call launch by bash script:
chmod 777 xxx.sh
sudo ./xxx.sh
```

7. record and replay
Recording and playing back data
https://docs.ros.org/en/foxy/Tutorials/Ros2bag/Recording-And-Playing-Back-Data.html

8. data visualization with RViz
   http://wiki.ros.org/rviz/UserGuide
   
9. rqt
Using rqt Tools for Analysis:https://industrial-training-master.readthedocs.io/en/melodic/_source/session6/Using-rqt-tools-for-analysis.html
- rqt_graph -->arch
  use rqt_graph to visualize the changing nodes and topics, as well as the connections between them.
  https://roboticsbackend.com/rqt-graph-visualize-and-debug-your-ros-graph/
- rqt_console -->log in GUI
  https://docs.ros.org/en/foxy/Tutorials/Rqt-Console/Using-Rqt-Console.html?highlight=rqt_console
- rqt_plot -->
  https://roboticsbackend.com/rqt-plot-easily-debug-ros-topics/

10. ros2 develop use c++
    - Writing a simple publisher and subscriber --> implement node code
    https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
    For the publisher node, spinning meant starting the timer, but for the subscriber it simply means preparing to receive messages whenever they come
      - configure
      - advertiseOutputs() --> create_publisher<messages>(topic,...)
      - subscribeToInputs() --> create_subscription<messages>(topic,rclQoS,getInputCallback)
      - runTask() --> create_wall_timer(cycle_time,publishCallback)
        --> fuse()
        --> publish()
    rclcpp:https://docs.ros.org/en/foxy/Tutorials/Tf2/Adding-A-Frame-Cpp.html
    - ament_cmake user documentation --> build
    https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html?highlight=cmake
    - Creating a launch file(python) --> launch muti-nodes
    https://docs.ros.org/en/foxy/Tutorials/Launch-Files/Creating-Launch-Files.html#write-the-launch-file

11. debug ros2 execuable
**Config:**
```// CMakeLists.txt
set(CMAKE_BUILD_TYPE Debug)
```

```// launch.json
{
  "configurations": [
      {
          "name": "ros2_publisher",
          "program": "${workspaceFolder}/install/examples_rclcpp_minimal_subscriber/lib/examples_rclcpp_minimal_subscriber/"
      }
  ]

  "compounds": [
      {
          "name": "ros2_pub_sub",
          "configurations": ["ros2_publisher", "ros2_subscriber"]
      }

  ]
}
```
**Hints:**
- set break point after node is running
**More about debuger**
- https://code.visualstudio.com/docs/editor/debugging
- https://github.com/ms-iot/vscode-ros/blob/master/doc/debug-support.md#attach


11. practices  
- 1. ROS2 Basics Exercise --> ros2 basic command usage
  https://industrial-training-master.readthedocs.io/en/melodic/_source/session7/ROS2-Basics.html
- 2. T2TF Demo:--> 