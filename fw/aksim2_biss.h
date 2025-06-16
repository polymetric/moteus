// Copyright 2023 Eli Rutan  polymetricofficial@gmail.com
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

#include "fw/stm32_spi.h"
#include "fw/aux_common.h"

namespace moteus {
class Aksim2Biss {
 public:
  using Options = Stm32Spi::Options;

  Aksim2Biss(const Options& options)
      : spi_([&]() {
               auto options_copy = options;
               options_copy.width = 8;
               options_copy.mode = 2;
               return options_copy;
             }()) {
  }

  void ISR_StartSample() /*MOTEUS_CCM_ATTRIBUTE*/ {
    StartDma(5);
  }

  void ISR_MaybeFinishSampe(aux::Spi::Status *status) MOTEUS_CCM_ATTRIBUTE {
    if (!spi_.is_dma_finished()) { return; }
    spi_.finish_dma_transfer();

    uint64_t crc_calc_input =
      ((((uint64_t) rx_buffer_[2]) & 0x7f) << 13)
      | (((uint64_t) rx_buffer_[3]) << 5)
      | (((uint64_t) rx_buffer_[4]) >> 3);
    uint8_t crc_calc = CRC_BiSS_43_42bit(crc_calc_input);
    uint8_t crc_sent = ((rx_buffer_[4] & 0x07) << 3) | rx_buffer_[5] >> 5;

    if (crc_calc == crc_sent) {
      status->active = true;
      status->value = (
        (static_cast<uint32_t>(rx_buffer_[2]) << 17) |
        (static_cast<uint32_t>(rx_buffer_[3]) << 9) |
        (static_cast<uint32_t>(rx_buffer_[4]) << 1)
      ) >> 6;
      // Status bits are active low
      status->aksim2_err = !(rx_buffer_[4] & 0x10);
      status->aksim2_warn = !(rx_buffer_[4] & 0x08);
      status->nonce += 1;
    } else {
      status->checksum_errors++;
    }
  }

 private:
  void StartDma(int size) MOTEUS_CCM_ATTRIBUTE {
    spi_.start_dma_transfer(
        std::string_view(
            reinterpret_cast<const char*>(&tx_buffer_[0]), size),
        mjlib::base::string_span(
            reinterpret_cast<char *>(&rx_buffer_[0]), size));
  }

  uint8_t ab_CRC6_LUT[64] = { 0x00, 0x03, 0x06, 0x05, 0x0C, 0x0F, 0x0A, 0x09,
    0x18, 0x1B, 0x1E, 0x1D, 0x14, 0x17, 0x12, 0x11, 0x30, 0x33, 0x36, 0x35,
    0x3C, 0x3F, 0x3A, 0x39, 0x28, 0x2B, 0x2E, 0x2D, 0x24, 0x27, 0x22, 0x21,
    0x23, 0x20, 0x25, 0x26, 0x2F, 0x2C, 0x29, 0x2A, 0x3B, 0x38, 0x3D, 0x3E,
    0x37, 0x34, 0x31, 0x32, 0x13, 0x10, 0x15, 0x16, 0x1F, 0x1C, 0x19, 0x1A,
    0x0B, 0x08, 0x0D, 0x0E, 0x07, 0x04, 0x01, 0x02
  };

  /*64-bit input data, right alignment, Calculation over 42 bits (mult. of 6) */
  uint8_t CRC_BiSS_43_42bit(uint64_t dw_InputData) {
    uint8_t b_Index = 0;
    uint8_t b_CRC = 0;
    b_Index = (uint8_t) ((dw_InputData >> 36u) & (uint64_t) 0x00000003Fu);
    b_CRC = (uint8_t) ((dw_InputData >> 30u) & (uint64_t) 0x0000003Fu);
    b_Index = b_CRC ^ ab_CRC6_LUT[b_Index];
    b_CRC = (uint8_t) ((dw_InputData >> 24u) & (uint64_t) 0x0000003Fu);
    b_Index = b_CRC ^ ab_CRC6_LUT[b_Index];
    b_CRC = (uint8_t) ((dw_InputData >> 18u) & (uint64_t) 0x0000003Fu);
    b_Index = b_CRC ^ ab_CRC6_LUT[b_Index];
    b_CRC = (uint8_t) ((dw_InputData >> 12u) & (uint64_t) 0x0000003Fu);
    b_Index = b_CRC ^ ab_CRC6_LUT[b_Index];
    b_CRC = (uint8_t) ((dw_InputData >> 6u) & (uint64_t) 0x0000003Fu);
    b_Index = b_CRC ^ ab_CRC6_LUT[b_Index];
    b_CRC = (uint8_t) (dw_InputData & (uint64_t) 0x0000003Fu);
    b_Index = b_CRC ^ ab_CRC6_LUT[b_Index];
    b_CRC = ab_CRC6_LUT[b_Index];
    return (~b_CRC) & 0x3F;
  }

  const uint8_t tx_buffer_[5] = {};
  uint8_t rx_buffer_[5] = {};

  Stm32Spi spi_;
};
}
