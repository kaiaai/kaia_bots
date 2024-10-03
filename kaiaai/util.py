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
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue


class ParamClient(Node):

  def __init__(self, node_name, wait_for_service=True):
    super().__init__('param_client' + node_name.replace('/', '_'))

    get_params_service_name = node_name + '/get_parameters'
    self.getter = self.create_client(GetParameters, get_params_service_name)

    set_params_service_name = node_name + '/set_parameters'
    self.setter = self.create_client(SetParameters, set_params_service_name)

    msg_waiting = ' not available, waiting ...'
    service_timeout_sec = 5.0

    if wait_for_service:
      while not self.getter.wait_for_service(timeout_sec=service_timeout_sec):
        self.get_logger().info(get_params_service_name + msg_waiting)

      while not self.setter.wait_for_service(timeout_sec=service_timeout_sec):
        self.get_logger().info(set_params_service_name + msg_waiting)

    self.get_req = GetParameters.Request()
    self.set_req = SetParameters.Request()

  def wait_get_service(timeout_sec=0.001):
    return self.getter.wait_for_service(timeout_sec=service_timeout_sec)

  def wait_set_service(timeout_sec=0.001):
    return self.setter.wait_for_service(timeout_sec=service_timeout_sec)

  def get(self, param_name):
    if not isinstance(param_name, list):
      param_name = [param_name]

    self.get_req.names = param_name

    self.future = self.getter.call_async(self.get_req)
    rclpy.spin_until_future_complete(self, self.future)
    return self.future.result()

  def set(self, param_name, param_value):
    if not isinstance(param_name, list):
      param_name = [param_name]

    if not isinstance(param_value, list):
      param_value = [param_value]

    for name, value in zip(param_name, param_value):
      param = Parameter()

      if isinstance(value, float):
        val = ParameterValue(double_value=value, type=ParameterType.PARAMETER_DOUBLE)
      elif isinstance(value, int):
        val = ParameterValue(integer_value=value, type=ParameterType.PARAMETER_INTEGER)
      elif isinstance(value, str):
        val = ParameterValue(string_value=value, type=ParameterType.PARAMETER_STRING)
      elif isinstance(value, bool):
        val = ParameterValue(bool_value=value, type=ParameterType.PARAMETER_BOOL)

      self.set_req.parameters.append(Parameter(name=name, value=val))

    self.future = self.setter.call_async(self.set_req)
    rclpy.spin_until_future_complete(self, self.future)
    return self.future.result()

  @staticmethod
  def to_value(response):

    val = []

    for value in response.values:

      if value.type == ParameterType.PARAMETER_DOUBLE:
        val.append(value.double_value)
      elif value.type == ParameterType.PARAMETER_INTEGER:
        val.append(value.integer_value)
      elif value.type == ParameterType.PARAMETER_STRING:
        val.append(value.string_value)
      elif value.type == ParameterType.PARAMETER_BOOL:
        val.append(value.bool_value)
      else:
        val.append(None)

    return val
