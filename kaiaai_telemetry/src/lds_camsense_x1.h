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
#include "lds.h"

class LDS_CamsenseX1 : public LDS
{
protected:
  static const uint8_t START_BYTE0 = 0x55;
  static const uint8_t START_BYTE1 = 0xAA;
  static const uint8_t START_BYTE2 = 0x03;
  static const uint8_t SAMPLES_PER_PACKET = 0x08;
  static const uint16_t ANGLE_MIN = 0xA000;

  struct meas_sample_t {
    int16_t distance_mm;
    uint8_t quality;
  } __attribute__((packed));

  struct scan_packet_t {
    uint8_t start_byte0; // 0x55
    uint8_t start_byte1; // 0xAA
    uint8_t start_byte2; // 0x03
    uint8_t samples_per_packet; // 0x08
    uint16_t rotation_speed; // Hz*64*60
    uint16_t start_angle;
    meas_sample_t sample[SAMPLES_PER_PACKET];
    uint16_t end_angle;
    uint16_t crc16;
  } __attribute__((packed));

  uint16_t rotation_speed;
  scan_packet_t scan_packet;
  uint16_t parser_idx;
  uint16_t end_angle_prev;

public:
  LDS_CamsenseX1() : LDS()
  {
    rotation_speed = 0;
    parser_idx = 0;
    end_angle_prev = 0;
  }

  static const std::string get_model_name() { return "CAMSENSE-X1"; }

  virtual float get_scan_time() override {
    return rotation_speed == 0 ? 0 : 360.0 * 3840 / rotation_speed;
  }

  uint16_t decodeUInt16(const uint16_t value) const {
    union {
      uint16_t i;
      char c[2];
    } bint = {0x0201};

    return bint.c[0] == 0x01 ? value : (value << 8) + (value >> 8);
  }

  virtual LDS::result_t decode_data(const void * context) override
  {
    LDS::result_t result = RESULT_OK;

    int current_byte = readByte(context);
    if (current_byte < 0)
      return RESULT_NOT_READY;
    uint8_t c = (uint8_t) current_byte;

    uint8_t * rx_buffer = (uint8_t *)&scan_packet;

    if (parser_idx >= sizeof(scan_packet_t)) {
      parser_idx = 0;
      return RESULT_OK;
    }

    rx_buffer[parser_idx++] = c;

    switch (parser_idx) {
    case 1:
      if (c != START_BYTE0) {
        parser_idx = 0;
      }
      break;

    case 2:
      if (c != START_BYTE1) {
        parser_idx = 0;
      }
      break;

    case 3:
      if (c != START_BYTE2) {
        parser_idx = 0;
      }
      break;

    case 4:
      if (c != SAMPLES_PER_PACKET) {
        result = RESULT_INVALID_PACKET;
      }
      break;

    case 5: // speed LSB
    case 6: // speed MSB
      break;

    case 7: // start angle LSB
    case 8: // start angle MSB
      break;

    default:
      if (parser_idx > sizeof(scan_packet_t)) {
        result = RESULT_INVALID_PACKET;
      }
      break;

    case sizeof(scan_packet_t) - 3: // end angle LSB
    case sizeof(scan_packet_t) - 2: // end angle MSB
      break;

    case sizeof(scan_packet_t) - 1: // CRC16 LSB
      break;

    case sizeof(scan_packet_t) - 0: // CRC16 MSB

      rotation_speed = decodeUInt16(scan_packet.rotation_speed);
      uint16_t start_angle = decodeUInt16(scan_packet.start_angle);
      uint16_t end_angle = decodeUInt16(scan_packet.end_angle);

      if (start_angle < ANGLE_MIN || end_angle < ANGLE_MIN) {
        result = RESULT_INVALID_PACKET;
        break;
      }

      bool scan_completed_mid_packet = end_angle < start_angle;
      bool scan_completed_between_packets = start_angle < end_angle_prev;
      bool scan_completed = scan_completed_mid_packet || scan_completed_between_packets;
      end_angle_prev = end_angle;

      static float constexpr ONE_OVER_64 = 1.0 / 64;
      float start_angle_deg = (start_angle - ANGLE_MIN) * ONE_OVER_64;
      float end_angle_deg = (end_angle - ANGLE_MIN) * ONE_OVER_64;
      if (start_angle > end_angle)
        end_angle_deg += 360;

      static float constexpr ONE_OVER_7 = 1.0 / (SAMPLES_PER_PACKET - 1);
      float step_deg = end_angle_deg - start_angle_deg;
      step_deg *= ONE_OVER_7;

      float angle_deg_prev = start_angle_deg;
      for (uint8_t i = 0; i < SAMPLES_PER_PACKET; i++) {
        float distance_mm = (int16_t) decodeUInt16(scan_packet.sample[i].distance_mm);
        float quality = scan_packet.sample[i].quality;

        float angle_deg = start_angle_deg + step_deg * i;

        scan_completed = false;
        if (scan_completed_mid_packet) {
          scan_completed = (angle_deg >= 360 && angle_deg_prev < 360);
        } else if (scan_completed_between_packets) {
          scan_completed = (i == 0);
        }
        angle_deg_prev = angle_deg;

        angle_deg = angle_deg > 360 ? angle_deg - 360 : angle_deg;
        distance_mm = distance_mm < 0 ? 0 : distance_mm;

        postScanPoint(context, angle_deg, distance_mm, quality, 0, scan_completed);
      }
      parser_idx = 0;
      break;
    }

    if (result < RESULT_OK)
      parser_idx = 0;

    return result;
  }
};
