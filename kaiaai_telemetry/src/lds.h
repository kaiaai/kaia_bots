// Copyright 2024 REMAKE.AI, KAIA.AI, MAKERSPET.COM
//
// Based on
//   Copyright 2015 - 2018 EAI TEAM http://www.eaibot.com
//   https://github.com/EAIBOT/ydlidar_arduino works
//   https://github.com/YDLIDAR/ydlidar_arduino broken
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
#include <string>

class LDS
{
public:
  typedef void (*ScanPointCallback)(const void * context, float, float, float, float, bool);
  typedef int (*ReadByteCallback)(const void * context);

  enum result_t {
    RESULT_OK = 0,
    RESULT_TIMEOUT = -1,
    RESULT_INVALID_PACKET = -2,
    RESULT_CHECKSUM_ERROR = -3,
    RESULT_NOT_READY = -4,
    RESULT_NOT_CONFIGURED = -5,
  };

protected:
  ScanPointCallback scan_point_callback;
  ReadByteCallback read_byte_callback;

public:
  LDS()
  {
    scan_point_callback = NULL;
    read_byte_callback = NULL;
  }

  virtual ~LDS() {}
  virtual LDS::result_t decode_data(const void * context) = 0;
  virtual float get_scan_time() = 0;

  static const std::string get_model_name();

  void setScanPointCallback(ScanPointCallback scan_point_callback) {
    this->scan_point_callback = scan_point_callback;
  }

  void setReadByteCallback(ReadByteCallback read_byte_callback) {
    this->read_byte_callback = read_byte_callback;
  }

protected:
  void postScanPoint(const void * context, float angle_deg, float dist_mm,
    float quality, float intensity_lsb, bool scan_completed) {
    // dist_mm <=0 indicates invalid point
    if (scan_point_callback)
      scan_point_callback(context, angle_deg, dist_mm, quality, intensity_lsb, scan_completed);
  }

  int readByte(const void * context) {
    return (read_byte_callback) ? read_byte_callback(context) : RESULT_NOT_CONFIGURED;
  }
};
