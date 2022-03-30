# ROS2 BASIC
## ROS2 installation and extension:
- [install ROS foxy](https://docs.ros.org/en/foxy/Installation/Linux-Install-Debians.html)
  1. start by source command
    ```bash
    source /opt/ros/foxy/local_setup.bash
    ```
  2. set ros2 as default on linux
    ```bash
    echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
    ```
  3. check default system environment
    ```bash
    gedit ~/.bashrc
    ```
  4. print environment vairable
    ```bash
    printenv | grep -i ROS
    ```

- install extension packages
  ```bash
    sudo apt update
    sudo apt install python3-colcon-common-extensions
    sudo apt install python3-rosdep2
    pip3 install opencv-python
    pip3 install -U scikit-learn
    sudo apt install libyaml-cpp-dev 
    sudo apt install ~nros-foxy-rqt*
    sudo apt install ros-foxy-diagnostic-updater
    sudo apt-get install python-caktin-tools
    sudo apt-get install ros-foxy-ros2bag \
                        ros-foxy-rosbag2-converter-default-plugins \
                        ros-foxy-rosbag2-storage-default-plugins
  
  ```

## ROS2 hello world
- [Introducing turtlesim and rqt](https://docs.ros.org/en/foxy/Tutorials/Turtlesim/Introducing-Turtlesim.html)

## ROS2 system concept
(key words: Publisher/Subscriber/Topic//Nodes/Services/Quality of Service/ROS Client Library --> rclcpp)

1. node --> minumum application unit
   - node
   - point to point
   - 1 to multi

2. node communication model --> exchange nodes message
   
  ![alt text](Nodes-TopicandService.gif "Output from ros2")
  - `publisher-subscriber model`
    - topic(单向) --> [Understanding ROS 2 nodes](https://docs.ros.org/en/foxy/Tutorials/Understanding-ROS2-Nodes.html)
    - publiser
    - subscribtion
    - msg

  - `call-and-response model`
    - service(双向)
    - client
    - server
    - srv(request-response)

3. nodes communication mechanism
   - DDS (middleware)
   - intra_process (config node option, set "use_intra_process_comms")

4. integrate nodes as execuable, relationship with middleware and opteration system.
   
   ![alt text](ROS1-ROS2-architecture-for-DDS-approach-to-ROS-We-clarify-the-performance-of-the-data.png "Output from ros2")

   - rclcpp
     - [ROS2 internal interfaces](https://docs.ros.org/en/foxy/Concepts/About-Internal-Interfaces.html#internal-api-architecture-overview)
   - opteration system
     - executor
     - single/multi process
     - single/multi thread
  
## package
(key words: CMakeLists.txt, package.xml)
- [Creating your first ROS 2 package](https://docs.ros.org/en/foxy/Tutorials/Creating-Your-First-ROS2-Package.html)

## build(generate 3 folder: build, install, log)
- [Using colcon to build packages](https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html)

  ```bash
  # list available ros2 package
  colcon list
  colcon graph
  # build ros2 package
  colcon build --packages-select examples_xxx
  # clean build file
  rm -rf install log build
  ```

## source
>When colcon has completed building successfully, the output will be in the install directory. Before you can use any of the installed executables or libraries, you will need to add them to your path and library paths. colcon will have generated bash/bat files in the install directory to help setup the environment. These files will add all of the required elements to your path and library paths as well as provide any bash or shell commands exported by packages.  
```$ . install/setup.bash```

## run/launch
- Launch files allow you to start up and configure a number of executables containing ROS 2 nodes simultaneously.
  - [Creating a launch file](https://docs.ros.org/en/foxy/Tutorials/Launch-Files/Creating-Launch-Files.html)
  - [Different formats Launch file](https://docs.ros.org/en/foxy/How-To-Guides/Launch-file-different-formats.html)

- Alternatively, we can call launch command by bash script:
  ```bash
  # call launch by bash script:
  chmod 777 xxx.sh
  sudo ./xxx.sh
  ```

## record and replay
- [Recording and playing back data](https://docs.ros.org/en/foxy/Tutorials/Ros2bag/Recording-And-Playing-Back-Data.html)

## data 3D visualization with RViz
- [RVIZ User Guide](http://wiki.ros.org/rviz/UserGuide)
   
## rqt
- [Using rqt Tools for Analysis](https://industrial-training-master.readthedocs.io/en/melodic/_source/session6/Using-rqt-tools-for-analysis.html)
  - `rqt_graph` --> view node arch,use rqt_graph to visualize the changing nodes and topics, as well as the connections between them.  
    https://roboticsbackend.com/rqt-graph-visualize-and-debug-your-ros-graph/
  - `rqt_console` --> log in GUI  
    https://docs.ros.org/en/foxy/Tutorials/Rqt-Console/Using-Rqt-Console.html?highlight=rqt_console
  - `rqt_plot` --> time series visualization  
    https://roboticsbackend.com/rqt-plot-easily-debug-ros-topics/

## ros2 develop use c++
- [Code style and language versions](https://docs.ros.org/en/foxy/Contributing/Code-Style-Language-Versions.html)
- implement node code --> `xxx_node.cpp`  
   - [Writing a simple publisher and subscriber](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html) 
      >For the publisher node, spinning meant starting the timer, but for the subscriber it simply means preparing to receive messages whenever they come

  - code demo c++
    ```c++
    // 1.configure
    // 2.advertiseOutputs()
    create_publisher<msg>("topic",...)
    // create timer task
    create_wall_timer(cycle_time,timer_callback)
    // timer_callback implement
    timer_callback(...)
    {
        fuse()
        publisher_->publish(msg_instance)
    }
      
    // 3.subscribeToInputs() 
    create_subscription<msg>("topic",rclQoS,topic_callback)
    // topic_callback implement
    topic_callback()
    {
        rclcpp::Subscription<msg>::SharedPtr subscription_
    }
    ```
    
- implement ros2 package --> `package.xml`
  - [Creating your first ROS 2 package](https://docs.ros.org/en/foxy/Tutorials/Creating-Your-First-ROS2-Package.html) 
    
- implement build file --> `CMakeLists.txt`
  - [ament_cmake user documentation](https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html?highlight=cmake) 
    
- implement python script to launch muti-excuables -->  `xxx_.launch.py`
  - [Creating a launch file](https://docs.ros.org/en/foxy/Tutorials/Launch-Files/Creating-Launch-Files.html#write-the-launch-file) 
    

## debug ros2 execuable
- configure cmake file
  ```cmake
  # CMakeLists.txt
  set(CMAKE_BUILD_TYPE Debug)
  ```

- configure launch json
  ```json
  // launch.json
  {
    "configurations": [
        {
            "name": "ros2_publisher",
            "program": "${workspaceFolder}/install/examples_rclcpp_minimal_subscriber/lib/examples_rclcpp_minimal_subscriber/"
        }
    ],

    "compounds": [
        {
            "name": "ros2_pub_sub",
            "configurations": ["ros2_publisher", "ros2_subscriber"]
        }
    ]
  }
  ```
- Hints
  - set break point after node is running
- More about debuger
  - https://code.visualstudio.com/docs/editor/debugging
  - https://github.com/ms-iot/vscode-ros/blob/master/doc/debug-support.md#attach


## practices  
- [ROS2 Basics Exercise](https://industrial-training-master.readthedocs.io/en/melodic/_source/session7/ROS2-Basics.html) --> ros2 basic command usage
- Demo:--> 