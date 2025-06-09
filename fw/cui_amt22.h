// Copyright 2025 Eli Rutan.  polymetricofficial@gmail.com
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

#include "mbed.h"

#include "hal/spi_api.h"

#include "fw/ccm.h"
#include "fw/moteus_hw.h"
#include "fw/stm32_spi.h"
#include "fw/aux_common.h"


namespace moteus {

class CuiAmt22 {
 public:
  using Options = Stm32Spi::Options;

  CuiAmt22(const Options& options, MillisecondTimer* timer)
      : spi_([&]() {
               auto options_copy = options;
               options_copy.width = 8;
               options_copy.mode = 0;
               return options_copy;
             }()),
        cs_(std::in_place_t(), options.cs, 1),
        timer_(timer) {
  }

  /// @return true if we finished reading the sensor and updated the value
  bool ISR_Update(aux::Spi::Status *status) MOTEUS_CCM_ATTRIBUTE {
    uint16_t value = 0;
    cs_->clear();
    timer_->wait_us(AMT22_TIME_AFTER_CS);

    spi_.start_byte(0x00);
    buffer_[0] = spi_.finish_byte();
    timer_->wait_us(AMT22_TIME_BETWEEN_BYTES);

    spi_.start_byte(0x00);
    buffer_[1] = spi_.finish_byte();
    timer_->wait_us(AMT22_TIME_AFTER_CS);

    cs_->set();

    value =
      ((static_cast<uint16_t>(buffer_[0]) << 8)
      | static_cast<uint16_t>(buffer_[1]));

    // check parity
    if ((value >> 15 & 1) != !(
        (value >> 13 & 1) ^
        (value >> 11 & 1) ^
        (value >> 9  & 1) ^
        (value >> 7  & 1) ^
        (value >> 5  & 1) ^
        (value >> 3  & 1) ^
        (value >> 1  & 1))
      || (value >> 14 & 1) != !(
        (value >> 12 & 1) ^
        (value >> 10 & 1) ^
        (value >>  8 & 1) ^
        (value >>  6 & 1) ^
        (value >>  4 & 1) ^
        (value >>  2 & 1) ^
        (value >>  0 & 1))
    ) {
      // parity failed, just wait for the next sample
      status->checksum_errors++;
      return false;
    }

    status->value = value & 0x3fff;
    status->active = true;
    status->nonce++;
    return true;
  }

 private:
  Stm32Spi spi_;
  std::optional<Stm32DigitalOutput> cs_;
  MillisecondTimer* const timer_;

  uint32_t last_byte_finished_us_ = 0;
  uint8_t buffer_[2] = {};

  static constexpr uint32_t AMT22_TIME_BETWEEN_READS = 40;
  static constexpr uint32_t AMT22_TIME_BETWEEN_BYTES = 3;
  static constexpr uint32_t AMT22_TIME_AFTER_CS = 3;
};

}
