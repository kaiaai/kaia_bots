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
//
// Based on https://github.com/robopeak/rplidar_arduino

#pragma once
#include "lds.h"

class LDS_RPLidarA1 : public LDS
{
protected:
  struct node_info_t {
    uint8_t    sync_quality;
    uint16_t   angle_q6_checkbit;
    uint16_t   distance_q2;
  } __attribute__((packed));

  uint8_t recvPos;
  node_info_t node;

  static uint8_t const RESP_MEAS_CHECKBIT = (0x1<<0);
  static uint8_t const RESP_MEAS_SYNCBIT = (0x1<<0);
  static uint8_t const RESP_MEAS_QUALITY_SHIFT = 2;
  static uint8_t const RESP_MEAS_ANGLE_SHIFT = 1;

public:
  LDS_RPLidarA1() : LDS()
  {
    recvPos = 0;
  }

  static const std::string get_model_name() { return "SLAMTEC-RPLIDAR-A1"; }

  virtual float get_scan_time() override { return -1; }

  virtual LDS::result_t decode_data(const void * context) override
  {
    while (true) {
      if (recvPos >= sizeof(node_info_t))
        recvPos = 0;

      int current_byte = readByte(context);
      if (current_byte < 0)
        return RESULT_NOT_READY;

      switch (recvPos) {
      case 0:
        if (((current_byte >> 1) ^ current_byte) & 0x01)
          break;
        continue;
      case 1:
        if (current_byte & RESP_MEAS_CHECKBIT)
          break;
        recvPos = 0;
        if (((current_byte >> 1) ^ current_byte) & 0x01)
          break;
        continue;
      }

      uint8_t *nodebuf = (uint8_t*)&node;
      nodebuf[recvPos++] = current_byte;

      if (recvPos == sizeof(node_info_t)) {
        float distance_mm = node.distance_q2 * 0.25f;
        float angle_deg = (node.angle_q6_checkbit >> RESP_MEAS_ANGLE_SHIFT) * 0.015625f; // /64
        uint8_t quality = (node.sync_quality >> RESP_MEAS_QUALITY_SHIFT);
        bool scan_completed = (node.sync_quality & RESP_MEAS_SYNCBIT);

        postScanPoint(context, angle_deg, distance_mm, quality, 0, scan_completed);

        recvPos = 0;
        return RESULT_OK;
      }
    }
  }
};
