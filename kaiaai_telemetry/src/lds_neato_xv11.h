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
// Based on https://github.com/getSurreal/XV_Lidar_Controller

#pragma once
#include "lds.h"

class LDS_NeatoXV11 : public LDS
{
protected:
    // REF: https://github.com/Xevel/NXV11/wiki
    // The bit 7 of byte 1 seems to indicate that the distance could not be calculated.
    // It's interesting to see that when this bit is set, the second byte is always 80, and the values of the first byte seem to be
    // only 02, 03, 21, 25, 35 or 50... When it's 21, then the whole block is 21 80 XX XX, but for all the other values it's the
    // data block is YY 80 00 00 maybe it's a code to say what type of error ? (35 is preponderant, 21 seems to be when the beam is
    // interrupted by the supports of the cover) .
    static const uint8_t INVALID_DATA_FLAG = (1 << 7);   // Mask for byte 1 of each data quad "Invalid data"
    // The bit 6 of byte 1 is a warning when the reported strength is greatly inferior to what is expected at this distance.
    // This may happen when the material has a low reflectance (black material...), or when the dot does not have the expected
    // size or shape (porous material, transparent fabric, grid, edge of an object...), or maybe when there are parasitic
    // reflections (glass... ).
    static const uint8_t STRENGTH_WARNING_FLAG = (1 << 6);  // Mask for byte 1 of each data quat "Strength Warning"
    static const uint8_t CRC_ERROR_FLAG = (1 << 0);

  protected:
    int eState;
    float scan_rpm;

    static const unsigned char COMMAND = 0xFA;        // Start of new packet
    static const int INDEX_LO = 0xA0;                 // lowest index value
    static const int INDEX_HI = 0xF9;                 // highest index value

    static const uint16_t N_DATA_QUADS = 4;                // there are 4 groups of data elements
    static const uint16_t N_ELEMENTS_PER_QUAD = 4;         // viz., 0=distance LSB; 1=distance MSB; 2=sig LSB; 3=sig MSB

    // Offsets to bytes within 'Packet'
    static const uint16_t OFFSET_TO_START = 0;
    static const uint16_t OFFSET_TO_INDEX = OFFSET_TO_START + 1;
    static const uint16_t OFFSET_TO_SPEED_LSB = OFFSET_TO_INDEX + 1;
    static const uint16_t OFFSET_TO_SPEED_MSB = OFFSET_TO_SPEED_LSB + 1;
    static const uint16_t OFFSET_TO_4_DATA_READINGS = OFFSET_TO_SPEED_MSB + 1;
    static const uint16_t OFFSET_TO_CRC_L = OFFSET_TO_4_DATA_READINGS + (N_DATA_QUADS * N_ELEMENTS_PER_QUAD);
    static const uint16_t OFFSET_TO_CRC_M = OFFSET_TO_CRC_L + 1;
    static const uint16_t PACKET_LENGTH = OFFSET_TO_CRC_M + 1;  // 22 length of a complete packet
    // Offsets to the (4) elements of each of the (4) data quads
    static const uint16_t OFFSET_DATA_DISTANCE_LSB = 0;
    static const uint16_t OFFSET_DATA_DISTANCE_MSB = OFFSET_DATA_DISTANCE_LSB + 1;
    static const uint16_t OFFSET_DATA_SIGNAL_LSB = OFFSET_DATA_DISTANCE_MSB + 1;
    static const uint16_t OFFSET_DATA_SIGNAL_MSB = OFFSET_DATA_SIGNAL_LSB + 1;

    uint8_t Packet[PACKET_LENGTH];                 // an input packet
    uint16_t ixPacket;                          // index into 'Packet' array

    static const uint8_t BAD_DATA_MASK = (INVALID_DATA_FLAG | STRENGTH_WARNING_FLAG);

    enum e_state{
      eState_Find_COMMAND = 0, // 1st state: find 0xFA (COMMAND) in input stream
      eState_Build_Packet = 1, // 2nd state: build the packet
    };
    // static const uint8_t eState_Find_COMMAND = 0; // 1st state: find 0xFA (COMMAND) in input stream
    // static const uint8_t eState_Build_Packet = eState_Find_COMMAND + 1;  // 2nd state: build the packet
    uint16_t aryDist[N_DATA_QUADS];    // there are (4) distances, one for each data quad
                                       // so the maximum distance is 16383 mm (0x3FFF)
    uint16_t aryQuality[N_DATA_QUADS]; // same with 'quality'

public:
  LDS_NeatoXV11() : LDS()
  {
    clearVars();
    scan_rpm = 0;
  }

  float get_scan_time() override {
    return (scan_rpm > 0) ? 60/scan_rpm : -1;
  }
  static const std::string get_model_name() { return "NEATO-XV11"; }

  LDS::result_t decode_data(const void * context) override
  {
    return processByte(context, readByte(context));
  }

  void clearVars()
  {
    for (int ix = 0; ix < N_DATA_QUADS; ix++) {
      aryDist[ix] = 0;
      aryQuality[ix] = 0;
      //aryInvalidDataFlag[ix] = 0;
    }
    for (ixPacket = 0; ixPacket < PACKET_LENGTH; ixPacket++)  // clear out this packet
      Packet[ixPacket] = 0;
    ixPacket = 0;
    eState = eState_Find_COMMAND; // This packet is done -- look for next COMMAND byte
  }

  bool isValidPacket()
  {
    unsigned long chk32;
    unsigned long checksum;
    const int bytesToCheck = PACKET_LENGTH - 2;
    const int CalcCRC_Len = bytesToCheck / 2;
    unsigned int CalcCRC[CalcCRC_Len];

    uint8_t b1a, b1b, b2a, b2b;
    int ix;

    for (int ix = 0; ix < CalcCRC_Len; ix++)
      CalcCRC[ix] = 0;

    // Perform checksum validity test
    for (ix = 0; ix < bytesToCheck; ix += 2)
      CalcCRC[ix / 2] = Packet[ix] + ((Packet[ix + 1]) << 8);

    chk32 = 0;
    for (ix = 0; ix < CalcCRC_Len; ix++)
      chk32 = (chk32 << 1) + CalcCRC[ix];
    checksum = (chk32 & 0x7FFF) + (chk32 >> 15);
    checksum &= 0x7FFF;
    b1a = checksum & 0xFF;
    b1b = Packet[OFFSET_TO_CRC_L];
    b2a = checksum >> 8;
    b2b = Packet[OFFSET_TO_CRC_M];

    return ((b1a == b1b) && (b2a == b2b));
  }

  void processSignalStrength(int iQuad)
  {
    uint8_t dataL, dataM;
    aryQuality[iQuad] = 0;
    int iOffset = OFFSET_TO_4_DATA_READINGS + (iQuad * N_DATA_QUADS) + OFFSET_DATA_SIGNAL_LSB;
    dataL = Packet[iOffset];
    dataM = Packet[iOffset + 1];
    aryQuality[iQuad] = dataL | (dataM << 8);
  }

  uint8_t processDistance(int iQuad)
  {
    // Data 0 to Data 3 are the 4 readings. Each one is 4 bytes long, and organized as follows :
    //   byte 0 : <distance 7:0>
    //   byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8>
    //   byte 2 : <signal strength 7:0>
    //   byte 3 : <signal strength 15:8>
    // dist[] = sets distance to object in binary: ISbb bbbb bbbb bbbb
    // so maximum distance is 0x3FFF (16383 decimal) millimeters (mm)
    uint8_t dataL, dataM;
    aryDist[iQuad] = 0;                     // initialize
    int iOffset = OFFSET_TO_4_DATA_READINGS + (iQuad * N_DATA_QUADS) + OFFSET_DATA_DISTANCE_LSB;
    // byte 0 : <distance 7:0> (LSB)
    // byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8> (MSB)
    dataM = Packet[iOffset + 1];           // get MSB of distance data + flags
    if (dataM & BAD_DATA_MASK)             // if either INVALID_DATA_FLAG or STRENGTH_WARNING_FLAG is set...
      return dataM & BAD_DATA_MASK;        // ...then return non-zero
    dataL = Packet[iOffset];               // LSB of distance data
    aryDist[iQuad] = dataL | ((dataM & 0x3F) << 8);
    return 0;                              // okay
  }

  void processSpeed()
  {
    // Extract motor speed from packet - two bytes little-endian, equals RPM/64
    uint8_t scan_rph_low_byte = Packet[OFFSET_TO_SPEED_LSB];
    uint8_t scan_rph_high_byte = Packet[OFFSET_TO_SPEED_MSB];
    scan_rpm = float( (scan_rph_high_byte << 8) | scan_rph_low_byte ) / 64.0;
  }

  uint16_t processIndex()
  {
    // processIndex - Process the packet element 'index'
    // index is the index byte in the 90 packets, going from A0 (packet 0, readings 0 to 3) to F9
    //    (packet 89, readings 356 to 359).
    // Returns the first angle (of 4) in the current 'index' group
    uint16_t angle = 0;
    uint16_t data_4deg_index = Packet[OFFSET_TO_INDEX] - INDEX_LO;
    angle = data_4deg_index * N_DATA_QUADS;     // 1st angle in the set of 4

    return angle;
  }

  LDS::result_t processByte(const void *context, int inByte)
  {
    // Switch, based on 'eState':
    // State 1: We're scanning for 0xFA (COMMAND) in the input stream
    // State 2: Build a complete data packet
    LDS::result_t result = LDS::RESULT_OK;
    if (eState == eState_Find_COMMAND) {      // flush input until we get COMMAND byte
      if (inByte == COMMAND) {
        eState++;                                 // switch to 'build a packet' state
        Packet[ixPacket++] = inByte;              // store 1st byte of data into 'Packet'
      }
    } else {
      Packet[ixPacket++] = inByte;        // keep storing input into 'Packet'
      if (ixPacket == PACKET_LENGTH) {
        // we've got all the input bytes, so we're done building this packet
        if (isValidPacket()) {      // Check packet CRC
          uint8_t aryInvalidDataFlag[N_DATA_QUADS] = {0, 0, 0, 0}; // non-zero = INVALID_DATA_FLAG or STRENGTH_WARNING_FLAG is set

          // the first scan angle (of group of 4, based on 'index'), in degrees (0..359)
          // get the starting angle of this group (of 4), e.g., 0, 4, 8, 12, ...
          uint16_t startingAngle = processIndex();

          processSpeed();

          // process each of the (4) sets of data in the packet
          for (int ix = 0; ix < N_DATA_QUADS; ix++)   // process the distance
            aryInvalidDataFlag[ix] = processDistance(ix);
          for (int ix = 0; ix < N_DATA_QUADS; ix++) { // process the signal strength (quality)
            aryQuality[ix] = 0;
            if (aryInvalidDataFlag[ix] == 0)
              processSignalStrength(ix);
          }

          for (int ix = 0; ix < N_DATA_QUADS; ix++) {
            uint8_t err = aryInvalidDataFlag[ix] & BAD_DATA_MASK;

            int angle = startingAngle + ix;
            postScanPoint(context, angle, err ? 0 : aryDist[ix], aryQuality[ix], 0, angle == 0);
          }
        } else {
          // Bad packet
          result = RESULT_CHECKSUM_ERROR;
        }

        clearVars();
      }
    }
    return result;
  }

};
