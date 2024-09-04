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

class LDS_Delta2A : public LDS
{
protected:
  static const uint8_t START_BYTE = 0xAA;
  static const uint8_t PROTOCOL_VERSION = 0x01;
  static const uint8_t PACKET_TYPE = 0x61;
  static const uint8_t DATA_TYPE_RPM_AND_MEAS = 0xAD;
  static const uint8_t DATA_TYPE_RPM_ONLY = 0xAE;
  static const uint8_t PACKETS_PER_SCAN = 16;
  static constexpr float DEG_PER_PACKET = 360.0f / (float)PACKETS_PER_SCAN;
  static const uint8_t MAX_DATA_SAMPLES = 61; // 28

  struct meas_sample_t {
    uint8_t quality;
    uint16_t distance_mm_x4;
  };
  static const uint16_t MAX_DATA_BYTE_LEN = sizeof(meas_sample_t) * MAX_DATA_SAMPLES;

  struct scan_packet_t {
    uint8_t    start_byte; // 0xAA
    uint16_t   packet_length;
    uint8_t    protocol_version; // 0x01
    uint8_t    packet_type; // 0x61
    uint8_t    data_type; // 0xAE -> data_length -> scan_freq_x20 -> no data
                          // 0xAD -> data_length -> scan_freq_x20 -> data
    uint16_t   data_length; // n_samples = (data_length - 5)/3;
    uint8_t    scan_freq_x20;
    int16_t offset_angle_x100; // signed
    uint16_t start_angle_x100; // unsigned?

    meas_sample_t sample[MAX_DATA_SAMPLES];

    uint16_t   checksum;
  } __attribute__((packed));

  uint8_t parser_state;
  float scan_freq_hz;
  scan_packet_t scan_packet;
  uint16_t parser_idx;
  uint16_t checksum;

public:
  LDS_Delta2A() : LDS()
  {
    scan_freq_hz = 0;
    parser_state = 0;
    checksum = 0;
  }

  static const std::string get_model_name() { return "3IROBOTIX-DELTA-2A"; }

  virtual float get_scan_time() override {
    return (scan_freq_hz <= 0) ? 0 : 1.0f / scan_freq_hz;
  }

  uint16_t decodeUInt16(const uint16_t value) const {
    union {
      uint16_t i;
      char c[2];
    } bint = {0x0102};

    return bint.c[0] == 0x01 ? value : (value << 8) + (value >> 8);
  }

  virtual LDS::result_t decode_data(const void * context) override
  {
    uint16_t packet_length = 0;
    uint16_t data_length = 0;
    LDS::result_t result = RESULT_OK;

    int current_byte = readByte(context);
    if (current_byte < 0)
      return RESULT_NOT_READY;
    uint8_t c = (uint8_t) current_byte;

    uint8_t * rx_buffer = (uint8_t *)&scan_packet;


    if (parser_idx >= sizeof(scan_packet_t)) {
      parser_idx = 0;
      return RESULT_OK; //RESULT_INVALID_PACKET;
    }

    rx_buffer[parser_idx++] = c;
    checksum += c;

    switch (parser_idx) {
    case 1:
      if (c != START_BYTE) {
        parser_idx = 0;
        result = RESULT_OK; //RESULT_INVALID_PACKET;
      } else
        checksum = c;
      break;

    case 2:
      break;

    case 3:
      packet_length = decodeUInt16(scan_packet.packet_length);
      if (packet_length > sizeof(scan_packet_t) - sizeof(scan_packet.checksum))
        result = RESULT_INVALID_PACKET;
      break;

    case 4:
      if (c != PROTOCOL_VERSION)
        result = RESULT_INVALID_PACKET;
      break;

    case 5:
      if (c != PACKET_TYPE)
        result = RESULT_INVALID_PACKET;
      break;

    case 6:
      if (c != DATA_TYPE_RPM_AND_MEAS && c != DATA_TYPE_RPM_ONLY)
        result = RESULT_INVALID_PACKET;
      break;

    case 7: // data length MSB
      break;

    case 8: // data length LSB
      data_length = decodeUInt16(scan_packet.data_length);
      if (data_length == 0 || data_length > MAX_DATA_BYTE_LEN)
        result = RESULT_INVALID_PACKET;
      break;

    default:
      // Keep reading
      packet_length = decodeUInt16(scan_packet.packet_length);
      if (parser_idx != packet_length + 2)
        break;

      uint16_t pkt_checksum = (rx_buffer[parser_idx-2] << 8) + rx_buffer[parser_idx-1];

      pkt_checksum += rx_buffer[parser_idx-2];
      pkt_checksum += rx_buffer[parser_idx-1];
      if (checksum != pkt_checksum) {
        result = RESULT_CHECKSUM_ERROR;
        break;
      }

      scan_freq_hz = scan_packet.scan_freq_x20 * 0.05;

      if (scan_packet.data_type == DATA_TYPE_RPM_AND_MEAS) {
        uint16_t start_angle_x100 = decodeUInt16(scan_packet.start_angle_x100);
        bool scan_completed = start_angle_x100 == 0;

        data_length = decodeUInt16(scan_packet.data_length);
        if (data_length < 8) {
          result = RESULT_INVALID_PACKET;
          break;
        }

        uint16_t sample_count = (data_length - 5) / 3;
        if (sample_count > MAX_DATA_SAMPLES) {
          result = RESULT_INVALID_PACKET;
          break;
        }
        float start_angle = start_angle_x100 * 0.01;
        float coeff = DEG_PER_PACKET / (float)sample_count;
        for (uint16_t idx = 0; idx < sample_count; idx++) {
          float angle_deg = start_angle + idx * coeff;

          uint16_t distance_mm_x4 = decodeUInt16(scan_packet.sample[idx].distance_mm_x4);
          float distance_mm = distance_mm_x4 * 0.25;
          float quality = scan_packet.sample[idx].quality;
          postScanPoint(context, angle_deg, distance_mm, quality, 0, scan_completed);
          scan_completed = false;
        }
      }
      parser_idx = 0;
      break;
    }

    if (result < RESULT_OK)
      parser_idx = 0;

    return result;
  }
};
