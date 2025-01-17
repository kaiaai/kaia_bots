#!/usr/bin/env python
#
# Copyright 2024 KAIA.AI
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
import yaml, os
from pathlib import Path

CONFIG_FILE_NAME = ".kaiaai.yaml"


def get_config_path():
  return Path.home() / CONFIG_FILE_NAME


def load():
  path = get_config_path()
  if not os.path.exists(path):
    os.mknod(path)

  with open(path, 'r') as file:
    config = yaml.safe_load(file)
    return {} if config == None else config

def save(config):
  with open(get_config_path(), 'w') as file:
    yaml.dump(config, file)

def get_var(var_name):
  config = load()
  if var_name in config.keys():
    return config[var_name]
  else:
    return 'makerspet_mini' if (var_name == 'robot.model') else None

def set_var(var_name, var_value):
  config = load()
  config[var_name] = var_value
  save(config)
