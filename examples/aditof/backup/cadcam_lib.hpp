/*
 * SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
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

#include <memory>

#include "hololink/core/hololink.hpp"

#include <getopt.h>

#include <iostream>
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
    OUTPUT_1 = 0b0001, // for first camera
    OUTPUT_2 = 0b0010, // for another camera
    OUTPUT_3 = 0b0100,
    OUTPUT_4 = 0b1000,
    DEFAULT = 0b0000
};

class ADII2CExpander {
public:
    //static constexpr uint32_t I2C_EXPANDER_ADDRESS = 0b01110000;

    ADII2CExpander(std::shared_ptr<hololink::Hololink> hololink_, uint32_t i2c_bus, uint32_t expander_addr);
    void configure(I2CExpanderOutputEN output_en = I2CExpanderOutputEN::DEFAULT);
    void set_register(uint16_t register_, uint32_t value, std::optional<Timeout> timeout = std::nullopt);

private:
    std::shared_ptr<Hololink::I2c> i2c_;
    uint32_t expander_addr_;
};

class ADIGPIOCtl {
public:

    ADIGPIOCtl(std::shared_ptr<hololink::Hololink> hololink_, hololink::Metadata &channel_metadata, uint32_t pin);
    void configure_reset_low(uint32_t pin);
    void configure_reset_high(uint32_t pin);
    bool wait_for_low_and_set_high_profile(uint32_t pin);

private:
    std::shared_ptr<Hololink::GPIO> gpio_;
    uint32_t reset_pin_;
};

#if 0
class Adcam {
public:
    static constexpr uint8_t  ADCAM_I2C_BUS_ADDRESS      = 0x38;
    static constexpr uint8_t  EXPANDER_0_I2C_BUS_ADDRESS = 0x68;
    static constexpr uint8_t  EXPANDER_1_I2C_BUS_ADDRESS = 0x58;

    static constexpr uint32_t CAM_I2C_BUS = 0; // replace with your constant

    Adcam(HololinkChannel& hololink_channel,
        uint32_t hololink_i2c_controller_address,
        const ChannelMetadata& channel_metadata);

    // Utility helpers
    static std::vector<uint8_t> serialize_words_be(const std::vector<uint16_t>& words);
    static std::vector<uint16_t> format_registers(uint64_t reg);
    static std::vector<uint16_t> words(std::initializer_list<uint16_t> w);

    // I2C primitives
    void set_register(uint16_t reg16, uint8_t value, std::optional<Timeout> timeout = std::nullopt);
    bool set_register16_no_response(const std::vector<uint16_t>& reg_words,
                                    std::optional<Timeout> timeout = std::nullopt);
    std::vector<uint8_t> set_register16_response(const std::vector<uint16_t>& reg_words,
                                                size_t resp_len,
                                                std::optional<Timeout> timeout = Timeout{});
    bool set_register16_no_response(uint64_t reg_scalar, std::optional<Timeout> timeout = std::nullopt);
    std::vector<uint8_t> set_register16_response(uint64_t reg_scalar, size_t resp_len,
                                                std::optional<Timeout> timeout = Timeout{});

    // High-level API methods (ported from Python)
    std::vector<uint8_t> get_version();
    void set_mipi();
    void set_mode();
    void read_nvm_config();
    void get_status();
    void get_only_status();
    int  probe_adcam_adtf3175();
    std::vector<uint8_t> force_stop_burst_mode();
    std::vector<uint8_t> get_fw_version();
    void get_chip_status();
    void stream_on();
    void stream_off();
    void get_ChipID();
    void get_Status();
    std::vector<uint8_t> get_ClockContinuousMode();
    void adcam_reset_power_on();
    void adcam_hard_reset();

    // Converter interface – adapt to your pipeline
    struct Converter {
    virtual ~Converter() = default;
    virtual uint32_t receiver_start_byte() = 0;
    virtual size_t transmitted_line_bytes(csi::PixelFormat pf, int width) = 0;
    virtual size_t received_line_bytes(size_t transmitted) = 0;
    virtual void configure(uint32_t start_byte,
                            size_t rx_line_bytes,
                            int width, int height,
                            csi::PixelFormat pixel_format) = 0;
    };

    void configure_converter(Converter& converter);

    csi::PixelFormat pixel_format() const;
    csi::BayerFormat bayer_format() const;

    void start();
    void stop();

private:
    HololinkChannel& hololink_channel_;
    std::shared_ptr<Hololink> hololink_;
    std::shared_ptr<I2CController> i2c_;

    int width_{2560};
    int height_{512};
    int mode_{3};
    csi::PixelFormat pixel_format_{csi::PixelFormat::RAW_8};

    ADII2CExpander expander0_;
    ADII2CExpander expander1_;
    ADIGPIOCtl pf_gpio_;

    std::vector<uint8_t> get_status_return();

    static void sleep_ms(int ms);
    static uint64_t time_now_s();
    static std::string to_hex(const std::vector<uint8_t>& data);
};
#else
/// ADCAM (ADTF3175 / ADSD3500 controller) C++ port of your Python `adcam`.
class Adcam {
 public:
  static constexpr uint8_t ADCAM_I2C_BUS_ADDRESS = 0x38;
  static constexpr uint8_t EXPANDER_0_I2C_BUS_ADDRESS = 0x68;
  static constexpr uint8_t EXPANDER_1_I2C_BUS_ADDRESS = 0x58;

  /// hololink_i2c_controller_address: same concept as Python hololink.get_i2c(addr)
  /// channel_metadata: opaque pointer/struct in your system (keep as void* until you bind actual type)
  Adcam(hololink::DataChannel& hololink_channel,
        uint32_t hololink_i2c_controller_address,
        hololink::Metadata &channel_metadata);

  // ---- High level ops (same names as your Python) ----
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
                        //    hololink::Hololink* hololink,
                        //     hololink::Channel* hololink_channel,
                        //     void* channel_metadata);

  void adcam_hard_reset();
                        // hololink::Hololink* hololink,
                        // hololink::Channel* hololink_channel,
                        // void* channel_metadata);

void profile_fpga_perf(uint32_t pin);                        
  // ---- Low-level register ops ----
  void set_register(uint16_t reg, uint8_t value);

  /// “register” may be larger than 16-bit in Python; we accept 64-bit and also arbitrary bytes.
  void set_register16_no_response(uint16_t *register_blob);

  std::vector<uint8_t> set_register16_response(uint16_t *register_blob, size_t resp_len);
  std::vector<uint8_t> set_register16_response(std::shared_ptr<hololink::sensors::Adcam> adcam_inst, uint16_t *register_blob, size_t resp_len);

  // ---- Helpers (ported) ----
  static std::vector<uint16_t> bytes_to_uint16_array_be(const std::vector<uint8_t>& data);
  static std::vector<uint8_t> registers_to_byte_array_be(uint64_t reg_blob);
  static std::vector<uint16_t> format_registers_be(uint64_t reg_blob);
  void configure_converter(std::shared_ptr<hololink::csi::CsiConverter> converter);
  uint32_t get_width();
  uint32_t get_height();
  void start(void);
  void stop(void);  

 private:
  // Adapter: build a big-endian uint16 stream to send over I2C
  static void append_u16_be(std::vector<uint8_t>& out, uint16_t v);

  // Adapter: talk to hololink I2C
  void i2c_write_read(const std::vector<uint8_t>& write_bytes,
                      size_t read_byte_count,
                      std::vector<uint8_t>* out_reply);

  void i2c_write_read(std::shared_ptr<hololink::sensors::Adcam> adcam_inst, const std::vector<uint8_t>& write_bytes,
                           size_t read_byte_count,
                           std::vector<uint8_t>* out_reply);

 private:
  std::shared_ptr<hololink::Hololink> hololink_{nullptr};
  std::shared_ptr<Hololink::I2c>  i2c_{nullptr};
  //std::shared_ptr<Hololink::I2c> i2c_;

  int width_{2560};
  int height_{512};
  int mode_{3};
  int pixel_format_{0};//{hololink_module.sensors.csi.PixelFormat.RAW_8};
  int test_{0};

  // expanders + gpio (as in python)
    ADII2CExpander expander0_;
    ADII2CExpander expander1_;
    ADIGPIOCtl pf_gpio_;
};
#endif

} // namespace hololink::sensors

#endif /* SENSORS_ADCAM_LIB_HPP */
