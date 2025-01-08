#!/usr/bin/env python3
#
# Copyright 2023-2025 KAIA.AI
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
import yaml
import os
import math
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from kaiaai import config
from ament_index_python.packages import get_package_share_path
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class MapPose(Node):
  def __init__(self):
    super().__init__('kaiaai_utils_MapPose')
    self.frame_from = 'base_footprint'
    self.frame_to = 'map'
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)
    self.current_x = 0.0
    self.current_y = 0.0
    self.current_yaw = 0.0

  def get_map_pos_2d(self):
    tf = None
    try:
      now = rclpy.time.Time()
      tf = self.tf_buffer.lookup_transform(self.frame_to, self.frame_from, now)
    except TransformException as ex:
#     self.get_logger().info(f'Could not transform {self.frame_from} to {self.frame_from}: {ex}')
      return None

    x = tf.transform.translation.x
    y = tf.transform.translation.y
    roll, pitch, yaw = self.euler_from_quaternion(tf.transform.rotation)

    return [x, y, yaw]

  @staticmethod
  def euler_from_quaternion(r):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (r.w * r.x + r.y * r.z)
    t1 = +1.0 - 2.0 * (r.x * r.x + r.y * r.y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (r.w * r.y - r.z * r.x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (r.w * r.z + r.x * r.y)
    t4 = +1.0 - 2.0 * (r.y * r.y + r.z * r.z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


class ModelParams():
  def __init__(self):
    robot_model_str = config.get_var('robot.model')

    description_package_path = get_package_share_path(robot_model_str)
    kaiaai_path_name = os.path.join(
      description_package_path,
      'config',
      'kaiaai.yaml'
    )

    with open(kaiaai_path_name, 'r') as stream:
      try:
        self.params = yaml.safe_load(stream)
      except yaml.YAMLError as exc:
        print(exc)

    # urdf_path_name = os.path.join(
    #   description_package_path,
    #   'urdf',
    #   'robot.urdf.xacro')

  def get_params(self):
    return self.params


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
