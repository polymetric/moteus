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
    StartDma(6);
  }

  void ISR_MaybeFinishSample(aux::Spi::Status *status) /*MOTEUS_CCM_ATTRIBUTE*/ {
    spi_.finish_dma_transfer();

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
  }

 private:
  void StartDma(int size) /*MOTEUS_CCM_ATTRIBUTE*/ {
    spi_.start_dma_transfer(
        std::string_view(
            reinterpret_cast<const char*>(&tx_buffer_[0]), size),
        mjlib::base::string_span(
            reinterpret_cast<char *>(&rx_buffer_[0]), size));
  }

  const uint8_t tx_buffer_[6] = {};
  uint8_t rx_buffer_[6] = {};

  Stm32Spi spi_;
};
}
