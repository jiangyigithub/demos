# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from example_interfaces.srv import AddTwoInts

import rclpy


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('minimal_client')
    # client的创建 --> not include callback
    cli = node.create_client(AddTwoInts, 'add_two_ints')
    # request input from client side
    req = AddTwoInts.Request()
    req.a = 4
    req.b = 1
    # 等待服务
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')
    # 发送请求 request --> response
    future = cli.call_async(req)
    # 等待响应
    rclpy.spin_until_future_complete(node, future)

    result = future.result() # get response
    node.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (req.a, req.b, result.sum))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
