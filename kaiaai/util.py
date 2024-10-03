#!/usr/bin/env python3
#
# Copyright 2023-2024 KAIA.AI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterType


class ParamClient(Node):

  def __init__(self, node_name, wait_for_service=true):
    super().__init__('param_client')

    get_params_service_name = node_name + '/get_parameters'
    self.geter = self.create_client(GetParameters, get_params_service_name)

    set_params_service_name = node_name + '/set_parameters'
    self.setter = self.create_client(SetParameters, set_params_service_name)

    if wait_for_service:
      while not self.getter.wait_for_service(timeout_sec=service_timeout_sec):
        self.get_logger().info(get_params_service_name + ' not available, waiting ...')

      while not self.setter.wait_for_service(timeout_sec=service_timeout_sec):
        self.get_logger().info(set_params_service_name + ' not available, waiting ...')

    self.get_req = GetParameters.Request()
    self.set_req = SetParameters.Request()

  def is_available(timeout_sec=0.001):
    return self.getter.wait_for_service(timeout_sec=service_timeout_sec)

  def get(self, params_name_list):
    self.get_req.names = params_name_list

    self.future = self.getter.call_async(self.req)
    rclpy.spin_until_future_complete(self, self.future)
    return self.future.result()


def main():
  rclpy.init()

  minimal_get_param_client = MinimalGetParamClientAsync()

  list_of_params_to_get = ['my_parameter', 'your_parameter']
  response = minimal_get_param_client.send_request(list_of_params_to_get)
  minimal_get_param_client.get_logger().info('First value: %s, Second value: %s'  % (response.values[0].string_value, response.values[1].string_value))

  minimal_get_param_client.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
