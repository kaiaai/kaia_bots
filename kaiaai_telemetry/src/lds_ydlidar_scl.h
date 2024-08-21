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
// Based on https://github.com/EAIBOT/ydlidar_arduino

#pragma once
#include "lds.h"

class LDS_YDLidarSCL : public LDS
{
protected:
  static const int PACKAGE_SAMPLE_MAX_LENGTH = 40;
  static const int PACKAGE_PAID_BYTES = 10;

  struct node_info_scl_t {
    float angle_deg;
    uint16_t distance_mm;
    uint8_t intensity;
    uint8_t quality_flag;
  } __attribute__((packed));

  struct cloud_point_scl_t {
    uint8_t intensity;
    uint8_t distance_lsb;
    uint8_t distance_msb;
  } __attribute__((packed));

  struct node_package_scl_t {
    uint16_t  package_Head;
    uint8_t   package_CT; // package type
    uint8_t   nowPackageNum; // distance sample count
    uint16_t  packageFirstSampleAngle;
    uint16_t  packageLastSampleAngle;
    uint16_t  checkSum;
    cloud_point_scl_t  packageSampleDistance[PACKAGE_SAMPLE_MAX_LENGTH];
  } __attribute__((packed));

protected:
  int recvPos;
  uint8_t package_Sample_Num;
  int package_recvPos;
  int package_sample_sum;
  int currentByte;

  node_package_scl_t package_scl;
  uint8_t *packageBuffer;

  uint16_t package_Sample_Index;
  float IntervalSampleAngle;
  float IntervalSampleAngle_LastPackage;
  uint16_t FirstSampleAngle;
  uint16_t LastSampleAngle;
  uint16_t CheckSum;
  uint16_t CheckSumCal;
  uint16_t SampleNumlAndCTCal;
  uint16_t LastSampleAngleCal;
  bool CheckSumResult;
  uint16_t Valu8Tou16;
  uint8_t state;

  uint8_t scan_freq;
  bool scan_completed;

public:
  LDS_YDLidarSCL() : LDS()
  {
    recvPos = 0;
    package_Sample_Num = 0;
    package_recvPos = 0;
    package_sample_sum = 0;
    currentByte = 0;

    packageBuffer = (uint8_t*) &package_scl.package_Head;

    package_Sample_Index = 0;
    IntervalSampleAngle = 0;
    IntervalSampleAngle_LastPackage = 0;
    FirstSampleAngle = 0;
    LastSampleAngle = 0;
    CheckSum = 0;
    CheckSumCal = 0;
    SampleNumlAndCTCal = 0;
    LastSampleAngleCal = 0;
    CheckSumResult = true;
    Valu8Tou16 = 0;
    state = 0;

    scan_freq = 0;
    scan_completed = false;
  }

  static const std::string get_model_name() { return "YDLIDAR-SCL"; }

  virtual float get_scan_time() override {
    return (scan_freq > 0) ? 10.0f/float(scan_freq) : -1;
  }

  virtual LDS::result_t decode_data(const void * context) override
  {
    switch(state) {
      case 1:
        goto state1;
      case 2:
        goto state2;
    }

    // Read in a packet; a packet contains up to 40 samples
    // Each packet has a Start and End (absolute) angles
    if (package_Sample_Index == 0) {

      // Read in, parse the packet header: first PACKAGE_PAID_BYTES=10 bytes
      package_Sample_Num = 0;
      package_recvPos = 0;
      recvPos = 0;

      while (true) {
state1:  // hack
        currentByte = readByte(context);

        if (currentByte < 0) {
          state = 1;
          return RESULT_NOT_READY;
        }

        switch (recvPos) {
        case 0:
          if (currentByte != 0xAA) { // (PH&0xFF)
            continue;
          }
          break;
        case 1:
          CheckSumCal = 0x55AA; // PH
          if (currentByte != 0x55) { // (PH>>8)
            recvPos = 0;
            continue;
          }
          break;
        case 2:
          SampleNumlAndCTCal = currentByte;
          break;
        case 3:
          SampleNumlAndCTCal += (currentByte << 8);
          package_Sample_Num = currentByte;
          break;
        case 4:
          if (currentByte & 0x01) {
            FirstSampleAngle = currentByte;
          } else {
            recvPos = 0;
            continue;
          }
          break;
        case 5:
          FirstSampleAngle += (currentByte << 8);
          CheckSumCal ^= FirstSampleAngle;
          FirstSampleAngle = FirstSampleAngle >> 1; // degrees*64
          break;
        case 6:
          if (currentByte & 0x01) {
            LastSampleAngle = currentByte;
          } else {
            recvPos = 0;
            continue;
          }
          break;
        case 7:
          LastSampleAngle += (currentByte << 8);
          LastSampleAngleCal = LastSampleAngle;
          LastSampleAngle = LastSampleAngle >> 1;
          if (package_Sample_Num == 1) {
            IntervalSampleAngle = 0;
          } else {
            if (LastSampleAngle < FirstSampleAngle) {
              if ((FirstSampleAngle > 270*64) && (LastSampleAngle < 90*64)){
                IntervalSampleAngle = ((float)(360*64 + LastSampleAngle - FirstSampleAngle))/(package_Sample_Num-1);
                IntervalSampleAngle_LastPackage = IntervalSampleAngle;
              } else{
                IntervalSampleAngle = IntervalSampleAngle_LastPackage;
              }
            } else {
              IntervalSampleAngle = ((float)(LastSampleAngle - FirstSampleAngle))/(package_Sample_Num-1);
              IntervalSampleAngle_LastPackage = IntervalSampleAngle;
            }
          }
          break;
        case 8:
          CheckSum = currentByte;
          break;
        case 9:
          CheckSum += (currentByte << 8);
          break;
        }
        packageBuffer[recvPos++] = currentByte;

        if (recvPos  == PACKAGE_PAID_BYTES) {
          package_recvPos = recvPos;
          break;
        }
      }

      // Check buffer overflow
      if (package_Sample_Num > PACKAGE_SAMPLE_MAX_LENGTH)
        return RESULT_INVALID_PACKET;

      // Read in the rest of the packet, i.e. samples
      if (PACKAGE_PAID_BYTES == recvPos) {
        recvPos = 0;
        package_sample_sum = package_Sample_Num * 3;

        while (true) {
state2:
          currentByte = readByte(context);
          if (currentByte < 0) {
            state = 2;
            return RESULT_NOT_READY;
          }

          switch (recvPos % 3) {
            case 0:
              CheckSumCal ^= currentByte;
              break;
            case 1:
              Valu8Tou16 = currentByte;
              break;
            case 2:
              Valu8Tou16 += (currentByte << 8);
              CheckSumCal ^= Valu8Tou16;
          }

          packageBuffer[package_recvPos + recvPos] = currentByte;
          recvPos++;
          if (package_sample_sum == recvPos) {
            package_recvPos += recvPos;
            break;
          }
        }

        if (package_sample_sum != recvPos) {
          state = 0;
          return RESULT_INVALID_PACKET;
        }
      } else {
        state = 0;
        return RESULT_INVALID_PACKET;
      }
      CheckSumCal ^= SampleNumlAndCTCal;
      CheckSumCal ^= LastSampleAngleCal;

      CheckSumResult = CheckSumCal == CheckSum;
    }

    scan_completed = false;
    if (CheckSumResult) {
      scan_completed = (package_scl.package_CT & 0x01) == 0x01;
      if (scan_completed)
        scan_freq = package_scl.package_CT >> 1;
    }

    while(true) {

      node_info_scl_t node;
      cloud_point_scl_t point;

      if (CheckSumResult == true) {
        int32_t AngleCorrectForDistance = 0;
        point = package_scl.packageSampleDistance[package_Sample_Index];
        node.distance_mm = (point.distance_lsb >> 2) + (point.distance_msb << 6);
        node.intensity = point.intensity;
        node.quality_flag = point.distance_lsb && 0x03;

        if (node.distance_mm != 0) {
          AngleCorrectForDistance = (int32_t)(atan(17.8f/((float)node.distance_mm))*3666.929888837269f);
          // *64*360/2/pi
        }

        float sampleAngle = IntervalSampleAngle * package_Sample_Index; // *64, before correction

        node.angle_deg = (FirstSampleAngle + sampleAngle + AngleCorrectForDistance) * 0.015625f; // /64

        if (node.angle_deg < 0) {
          node.angle_deg = node.angle_deg + 360;
        } else {
          if (node.angle_deg > 360) {
            node.angle_deg = node.angle_deg - 360;
          }
        }
      } else {
        node.angle_deg = 0;
        node.distance_mm = 0;
        node.intensity = 0;
        package_Sample_Index = 0;
        state = 0;
        return RESULT_CHECKSUM_ERROR;
      }

      // Dump out processed data
      postScanPoint(context, node.angle_deg, node.distance_mm, node.quality_flag, node.intensity, scan_completed);
      scan_completed = false;

      // Dump finished?
      package_Sample_Index++;
      uint8_t nowPackageNum = package_scl.nowPackageNum;
      if (package_Sample_Index >= nowPackageNum) {
        package_Sample_Index = 0;
        break;
      }
    }
    state = 0;

    return RESULT_OK;
  }

};
