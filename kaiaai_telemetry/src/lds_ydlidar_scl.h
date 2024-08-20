// Copyright 2024 REMAKE.AI, KAIA.AI, MAKERSPET.COM
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
#include "lds_ydlidar_x4.h"

class LDS_YDLidarSCL : public LDS_YDLidarX4
{
public:
  LDS_YDLidarSCL() : LDS_YDLidarX4() {}
  virtual float get_scan_time() override {
    return (scan_freq > 0) ? 10.0f/float(scan_freq) : -1;
  }
  static const std::string get_model_name() { return "YDLIDAR-SCL"; }
};
