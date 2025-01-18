#!/usr/bin/env python3
#
# Copyright 2023-2025 KAIA.AI
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
from nav2_msgs.srv import SaveMap
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from enum import Enum
import numpy as np
from PIL import Image
import yaml
from nav2_simple_commander.robot_navigator import TaskResult


class NavUtils(Node):
  def __init__(self, global_frame_id='map', base_frame_id='base_footprint'):
    super().__init__('kaiaai_utils')
    self.base_frame_id = base_frame_id
    self.global_frame_id = global_frame_id
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)
    self.map_saver_srv = self.create_client(SaveMap, 'map_server/save_map')
    self.map_srv = self.create_client(GetMap, 'slam_toolbox/dynamic_map')
    self.map_sub = self.create_subscription(OccupancyGrid(), '/map', self.mapCallback, 10)

  def saveMap(self, map_filepath='my_map', map_topic='map', image_format='pgm',
    map_mode='trinary', free_thresh=0.25, occupied_thresh=0.65):
    """Save the current static map to a file"""

    # Alternative: call /slam_toolbox/save_map
    while not self.map_saver_srv.wait_for_service(timeout_sec=1.0):
      self.info('map_saver/save_map service not available, waiting...')
    req = SaveMap.Request()
    req.map_url = map_filepath
    req.map_topic = map_topic
    req.image_format = image_format
    req.map_mode = map_mode
    req.free_thresh = free_thresh
    req.occupied_thresh = occupied_thresh

    future = self.map_saver_srv.call_async(req)
    rclpy.spin_until_future_complete(self, future)

    return future.result().result

  def mapCallback(self, msg):
    self.map = OccupancyGrid2d(msg)

  def getMap(self):
    return self.map

  def getCurrentMap(self):
    """Get the SLAM current OccupancyGrid"""
    # subscribe to /map topic (published by /slam_toolbox or /map_server)
    # call /map_server/get_map (if map_server is running)
    # call /slam_toolbox/dynamic_map nav_msgs/srv/GetMap

    # Alternative: call /slam_toolbox/save_map
    while not self.map_srv.wait_for_service(timeout_sec=1.0):
      self.info('slam_toolbox/get_map service not available, waiting...')

    req = GetMap.Request()
    future = self.map_srv.call_async(req)
    rclpy.spin_until_future_complete(self, future)

    map = future.result().map
    return OccupancyGrid2d(map)

  def getMapPos(self):

    tf = None
    while tf == None:
      tf = self.tryGetMapPos()
      rclpy.spin_once(self)

    return tf

  def getMapPos2d(self):

    tf = self.getMapPos()

    pos = dict()
    pos['x'] = tf.transform.translation.x
    pos['y'] = tf.transform.translation.y
    roll, pitch, yaw = self.euler_from_quaternion(tf.transform.rotation)
    pos['yaw'] = yaw

    return pos

  def tryGetMapPos(self):
    # Alternative: subscribe to /pose
    tf = None
    try:
      now = rclpy.time.Time()
      tf = self.tf_buffer.lookup_transform(self.global_frame_id, self.base_frame_id, now)
    except TransformException as ex:
      # self.get_logger().info(f'Could not transform {self.base_frame_id} to {self.global_frame_id}: {ex}')
      return None

    return tf

  def info(self, msg):
    self.get_logger().info(msg)
    return

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

  @staticmethod
  def getModelParams():
    robot_model_str = config.get_var('robot.model')

    description_package_path = get_package_share_path(robot_model_str)
    kaiaai_path_name = os.path.join(
      description_package_path,
      'config',
      'kaiaai.yaml'
    )

    with open(kaiaai_path_name, 'r') as stream:
      try:
        params = yaml.safe_load(stream)
      except yaml.YAMLError as exc:
        print(exc)

    # urdf_path_name = os.path.join(
    #   description_package_path,
    #   'urdf',
    #   'robot.urdf.xacro')

    return params

  @staticmethod
  def taskResultToText(result: TaskResult):
    if result == TaskResult.SUCCEEDED:
        return 'Goal succeeded'
    elif result == TaskResult.CANCELED:
        return 'Goal was canceled'
    elif result == TaskResult.FAILED:
        return 'Goal failed'
    else:
        return 'Goal has an invalid return status'


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


class OccupancyGrid2d():
  class CostValues(Enum):
    FreeSpace = 0
    InscribedInflated = 100
    LethalObstacle = 100
    NoInformation = -1

  def __init__(self, map):
    self.map = map

  @staticmethod
  def load(image_pathname='map.png'):
    img = Image.open(image_pathname)

    yaml_pathname = image_pathname + '.yaml'
    with open(yaml_pathname) as stream:
    # try:
      props = yaml.safe_load(stream)
    # except yaml.YAMLError as exc:
    #   print(exc)

    msg = OccupancyGrid()
    width, height = img.size
    msg.info.width = width
    msg.info.height = height
    msg.info.resolution = props['resolution']
    msg.header.frame_id = props['frame_id']

    position = props['origin']['position']
    msg.info.origin.position.x = position['x']
    msg.info.origin.position.y = position['y']
    # msg.info.origin.position.z = position['z']

    orientation = props['origin']['orientation']
    msg.info.origin.orientation.x = orientation['x']
    msg.info.origin.orientation.y = orientation['y']
    msg.info.origin.orientation.z = orientation['z']
    msg.info.origin.orientation.w = orientation['w']

    np_array = np.array(img)
    np_array = np_array.flatten()
    np_array = np_array.astype(np.int8)
    msg.data = np_array.tolist()

    map = OccupancyGrid2d(msg)
    return map

  def save(self, image_pathname='map.png'):
    data = np.array(self.map.data, dtype=np.uint8).reshape(self.getSizeY(), self.getSizeX())
    img = Image.fromarray(data, mode='L')
    img.save(image_pathname)

    orientation = self.getOrientation()
    data = dict(
      frame_id = self.map.header.frame_id,
      resolution = self.getResolution(),
      origin = dict(
        position = dict(
          x = self.getOriginX(),
          y = self.getOriginY(),
          # z = self.map.info.origin.position.z
          # yaw = self.getOriginYaw(),
        ),
        orientation = dict(
          x = orientation.x,
          y = orientation.y,
          z = orientation.z,
          w = orientation.w,
        ),
      ),
    )

    yaml_pathname = image_pathname + '.yaml'
    with open(yaml_pathname, 'w') as outfile:
      yaml.dump(data, outfile) # default_flow_style=False

  def getOrientation(self):
    return self.map.info.origin.orientation

  def getCost(self, mx, my):
    return self.map.data[self.__getIndex(mx, my)]

  def getSize(self):
    return (self.map.info.width, self.map.info.height)

  def getSizeX(self):
    return self.map.info.width

  def getSizeY(self):
    return self.map.info.height

  def getResolution(self):
    return self.map.info.resolution

  def getOriginX(self):
    return self.map.info.origin.position.x

  def getOriginY(self):
    return self.map.info.origin.position.y

  def getOriginYaw(self):
    roll, pitch, yaw = NavUtils.euler_from_quaternion(self.map.info.origin.orientation)
    return yaw

  def mapToWorld(self, mx, my):
    wx = self.map.info.origin.position.x + (mx + 0.5) * self.map.info.resolution
    wy = self.map.info.origin.position.y + (my + 0.5) * self.map.info.resolution

    return (wx, wy)

  def worldToMap(self, wx, wy):
    if (wx < self.map.info.origin.position.x or wy < self.map.info.origin.position.y):
      raise Exception("World coordinates out of bounds")

    mx = int((wx - self.map.info.origin.position.x) / self.map.info.resolution)
    my = int((wy - self.map.info.origin.position.y) / self.map.info.resolution)

    if (my > self.map.info.height or mx > self.map.info.width):
      raise Exception("Out of bounds")

    return (mx, my)

  def __getIndex(self, mx, my):
    return my * self.map.info.width + mx


class Costmap2d():
    class CostValues(Enum):
        FreeSpace = 0
        InscribedInflated = 253
        LethalObstacle = 254
        NoInformation = 255
    
    def __init__(self, map):
        self.map = map

    def getCost(self, mx, my):
        return self.map.data[self.__getIndex(mx, my)]

    def getSize(self):
        return (self.map.metadata.size_x, self.map.metadata.size_y)

    def getSizeX(self):
        return self.map.metadata.size_x

    def getSizeY(self):
        return self.map.metadata.size_y

    def __getIndex(self, mx, my):
        return my * self.map.metadata.size_x + mx