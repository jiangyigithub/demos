# repo
https://github.com/ros2/examples/tree/foxy/rclcpp/topics

# config
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

# build
``` bash
colcon build --packages-select examples_rclcpp_minimal_publisher
colcon build --packages-select examples_rclcpp_minimal_subscriber
```

# run
``` bash
ros2 run examples_rclcpp_minimal_publisher publisher_member_function # terminal 1
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function # termianl 2
```
# code deme
- Writing a simple publisher and subscriber --> implement node code
https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
```//comment
For the publisher node, spinning meant starting the timer, but for the subscriber it simply means preparing to receive messages whenever they come
```
    - configure
    - advertiseOutputs() --> create_publisher<messages>(topic,...)
    - subscribeToInputs() --> create_subscription<messages>(topic,rclQoS,getInputCallback)
    - runTask() --> create_wall_timer(cycle_time,publishCallback)
    --> fuse()
    --> publish()

# detail code
- https://blog.csdn.net/qq_16893195/article/details/112983147