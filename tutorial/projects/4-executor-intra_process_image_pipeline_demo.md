# Project: intra_process_image_pipeline_demo --> executor

## adapt opencv source from video
- https://blog.csdn.net/qq_16893195/article/details/113728729?spm=1001.2014.3001.5501

- video file: ./tutorial/data/chaplin.mp4

## dependency
``` bash
pip install opencv-python
pip3 install -U scikit-learn
sudo apt install python3-rosdep2
```

## include dependency in vscode
```c_cpp_properties.json
"/usr/include/opencv4/opencv2/core"
"/opt/ros/foxy/include/**"
```

## debug camera_node config
```CMakeLists.txt
set(CMAKE_BUILD_TYPE Debug)
```

```launch.json
program": "${workspaceFolder}/install/intra_process_demo/lib/intra_process_demo/camera_node
```

## build ros packages
```bash
colcon build --packages-select intra_process_demo
```

## ros run/launch
1. three nodes three process--> communication by RMW
```bash
ros2 run intra_process_demo image_view_node # terminal 1
ros2 run intra_process_demo watermark_node # terminal 2
ros2 run intra_process_demo camera_node #terminal 3
# or:
./install/intra_process_demo/lib/intra_process_demo/camera_node
```

2. tree nodes one process, intra-process for 1 publisher to 1 subscription--> zero copy by intra_process
```bash
ros2 run intra_process_demo image_pipeline_all_in_one
```

3. tree nodes one process, intra-process for 1 publiser to muti subscription--> copy n-1 data for 1 pub to n sub scenario
```bash
ros2 run intra_process_demo image_pipeline_with_two_image_view
```

## git file config
```.gitignore
build/
install/
log/
```

# code demo
```c++
// node constructor using intra_process
  WatermarkNode(
    const std::string & input, const std::string & output, const std::string & text,
    const std::string & node_name = "watermark_node")
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
  }
// single 
rclcpp::executors::SingleThreadedExecutor executor(...)
executor.add_node(Node xxx)
executor.spin()
// multi
rclcpp::executors::MultiThreadedExecutor executor(...)
executor.add_node(Node xxx)
executor.spin()
```

# reference
- https://docs.ros.org/en/foxy/Tutorials/Intra-Process-Communication.html?highlight=rclcpp%20executors
- https://blog.csdn.net/qq_16893195/article/details/113123386
- https://vimeo.com/292707644