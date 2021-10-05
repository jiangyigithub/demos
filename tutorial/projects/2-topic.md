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

# detail code
- https://blog.csdn.net/qq_16893195/article/details/112983147