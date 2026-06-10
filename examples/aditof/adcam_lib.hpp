/*
 * SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION
 * & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SENSORS_ADCAM_LIB_HPP
#define SENSORS_ADCAM_LIB_HPP

#include <cstdint>
#include <getopt.h>
#include <iostream>
#include <memory>
#include <string>

#include <hololink/common/cuda_helper.hpp>
#include <hololink/common/tools.hpp>
#include <hololink/core/data_channel.hpp>
#include <hololink/core/enumerator.hpp>
#include <hololink/core/hololink.hpp>
#include <hololink/core/logging.hpp>
#include <hololink/operators/csi_to_bayer/csi_to_bayer.hpp>
#include <hololink/operators/image_processor/image_processor.hpp>
#include <hololink/operators/roce_receiver/roce_receiver_op.hpp>

#include <holoscan/holoscan.hpp>
#include <holoscan/operators/bayer_demosaic/bayer_demosaic.hpp>
#include <holoscan/operators/holoviz/holoviz.hpp>

namespace hololink::sensors {

enum class I2CExpanderOutputEN : uint8_t {
  OUTPUT_1 = 0b0001,
  OUTPUT_2 = 0b0010,
  OUTPUT_3 = 0b0100,
  OUTPUT_4 = 0b1000,
  DEFAULT  = 0b0000
};

class ADII2CExpander {
 public:
  ADII2CExpander(std::shared_ptr<hololink::Hololink> hololink_,
                 uint32_t i2c_bus,
                 uint32_t expander_addr);

  void configure(I2CExpanderOutputEN output_en = I2CExpanderOutputEN::DEFAULT);

  void set_register(uint16_t register_,
                    uint32_t value,
                    std::optional<Timeout> timeout = std::nullopt);

 private:
  std::shared_ptr<Hololink::I2c> i2c_;
  uint32_t expander_addr_;
};

class ADIGPIOCtl {
 public:
  ADIGPIOCtl(std::shared_ptr<hololink::Hololink> hololink_,
             hololink::Metadata& channel_metadata,
             uint32_t pin);

  void configure_reset_low(uint32_t pin);
  void configure_reset_high(uint32_t pin);
  bool wait_for_low_and_set_high_profile(uint32_t pin);

 private:
  std::shared_ptr<Hololink::GPIO> gpio_;
  uint32_t reset_pin_;
};

/// ADCAM (ADTF3175 / ADSD3500 controller)
class Adcam {
 public:
  static constexpr uint8_t ADCAM_I2C_BUS_ADDRESS        = 0x38;
  static constexpr uint8_t EXPANDER_0_I2C_BUS_ADDRESS   = 0x68;
  static constexpr uint8_t EXPANDER_1_I2C_BUS_ADDRESS   = 0x58;

  Adcam(std::shared_ptr<hololink::DataChannel> hololink_channel,
        uint32_t hololink_i2c_controller_address,
        hololink::Metadata& channel_metadata,
        uint32_t adcam_mode,
        uint32_t reset_pin);

  ~Adcam();

  // ---- High-level ops ----
  std::vector<uint8_t> get_version();
  void set_mipi(std::shared_ptr<hololink::sensors::Adcam> adcam_inst);
  void set_mode();
  void read_nvm_config();
  void get_status();
  void get_only_status();

  int probe_adcam_adtf3175();
  std::vector<uint8_t> force_stop_burst_mode();
  std::vector<uint8_t> get_fw_version();
  void get_chip_status();

  void stream_on();
  void stream_off();

  void get_ChipID();
  void get_Status();
  std::vector<uint8_t> get_ClockContinuousMode();

  void adcam_reset_power_on();
  void adcam_Only_reset();

  void profile_fpga_perf(uint32_t pin);

  // ---- Low-level register ops ----
  void set_register(uint16_t reg, uint8_t value);

  void set_register16_no_response(uint16_t* register_blob);

  std::vector<uint8_t> set_register16_response(uint16_t* register_blob,
                                               size_t resp_len);

  std::vector<uint8_t> set_register16_response(
      std::shared_ptr<hololink::sensors::Adcam> adcam_inst,
      uint16_t* register_blob,
      size_t resp_len);

  // ---- Helpers ----
  static std::vector<uint16_t> bytes_to_uint16_array_be(
      const std::vector<uint8_t>& data);

  static std::vector<uint8_t> registers_to_byte_array_be(uint64_t reg_blob);
  static std::vector<uint16_t> format_registers_be(uint64_t reg_blob);

  void configure_converter(
      std::shared_ptr<hololink::csi::CsiConverter> converter);

  uint32_t get_width();
  uint32_t get_height();

  void start();
  void stop();

 private:
  static void append_u16_be(std::vector<uint8_t>& out, uint16_t v);

  void i2c_write_read(const std::vector<uint8_t>& write_bytes,
                      size_t read_byte_count,
                      std::vector<uint8_t>* out_reply);

  void i2c_write_read(
      std::shared_ptr<hololink::sensors::Adcam> adcam_inst,
      const std::vector<uint8_t>& write_bytes,
      size_t read_byte_count,
      std::vector<uint8_t>* out_reply);

 private:
  std::shared_ptr<hololink::Hololink> hololink_{nullptr};
  std::shared_ptr<Hololink::I2c> i2c_{nullptr};

  int width_{2560};
  int height_{512};
  int mode_{3};
  int pixel_format_{0};
  int test_{0};

  uint32_t adcam_mode_{6};
  uint32_t reset_pin_{0};

  // expanders + gpio
  ADII2CExpander expander0_;
  ADII2CExpander expander1_;
  ADIGPIOCtl pf_gpio_;
};

}  // namespace hololink::sensors

#endif /* SENSORS_ADCAM_LIB_HPP */
