/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "cadcam_lib.hpp"
#include <holoscan/logger/logger.hpp>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <algorithm>


#include <hololink/core/serializer.hpp>
#include <hololink/core/timeout.hpp>

namespace hololink::sensors {
ADII2CExpander::ADII2CExpander(std::shared_ptr<hololink::Hololink> hololink_, uint32_t i2c_bus, uint32_t expander_addr)
    : i2c_(hololink_->get_i2c(i2c_bus)), expander_addr_(expander_addr)
{
}

void ADII2CExpander::configure(I2CExpanderOutputEN output_en)
{
    std::vector<uint8_t> write_bytes(1);
    core::Serializer serializer(write_bytes.data(), write_bytes.size());
    serializer.append_uint8(static_cast<uint8_t>(output_en));

    uint32_t read_byte_count = 0;
    HOLOSCAN_LOG_DEBUG("ADCAM Expander Config");
    i2c_->i2c_transaction(expander_addr_, write_bytes, read_byte_count);
}

void ADII2CExpander::set_register(uint16_t register_, uint32_t value, std::optional<Timeout> timeout)
{
    std::vector<uint8_t> write_bytes(3);
    core::Serializer serializer(write_bytes.data(), write_bytes.size());
    //serializer.append_uint8(static_cast<uint8_t>(value));
    serializer.append_uint16_be(register_);
    serializer.append_uint8(value);    
    uint32_t read_byte_count = 0;
    HOLOSCAN_LOG_DEBUG("ADCAM Expander Config set {} value {}", expander_addr_, write_bytes);
    i2c_->i2c_transaction(expander_addr_, write_bytes, read_byte_count);
}

void ADIGPIOCtl::configure_reset_low(uint32_t pin)
{
    HOLOSCAN_LOG_DEBUG("Resetting ADCAM..set GPIO to LOW");
    gpio_->set_value(pin, gpio_->LOW);

}

void ADIGPIOCtl::configure_reset_high(uint32_t pin)
{
    HOLOSCAN_LOG_DEBUG("Resetting ADCAM..set GPIO to HIGH");
    gpio_->set_value(pin, gpio_->HIGH);
}

ADIGPIOCtl::ADIGPIOCtl(std::shared_ptr<hololink::Hololink> hololink_, hololink::Metadata &channel_metadata, uint32_t pin)
    : reset_pin_(pin)
{
    gpio_ = hololink_->get_gpio(channel_metadata);
}

bool ADIGPIOCtl::wait_for_low_and_set_high_profile(uint32_t pin)
{
    constexpr int poll_interval_us = 1;        // 10 microseconds
    constexpr int timeout_us = 1'000'000;       // 1 second timeout
    int elapsed_us = 0;
    uint64_t loop_count = 0;

    HOLOSCAN_LOG_INFO("Profiling GPIO {} for LOW → HIGH transition", pin);

    auto t_start = std::chrono::high_resolution_clock::now();
    gpio_->set_value(pin, gpio_->HIGH);

    while (elapsed_us < timeout_us) {
        loop_count++;

        // Read GPIO value
        auto value = gpio_->get_value(pin);

        if (value == gpio_->HIGH) {
            auto t_end = std::chrono::high_resolution_clock::now();
            gpio_->set_value(pin, gpio_->LOW);
            auto total_us = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();
            HOLOSCAN_LOG_INFO(
                "GPIO {} went LOW after {} µs ({} loops). Setting HIGH.",
                pin, total_us, loop_count
            );

            
            return true;
        }

        // Busy-wait for 10 microseconds
        auto t_poll = std::chrono::high_resolution_clock::now();
        while (true) {
            auto now = std::chrono::high_resolution_clock::now();
            auto diff = std::chrono::duration_cast<std::chrono::microseconds>(now - t_poll).count();
            if (diff >= poll_interval_us) break;
        }

        elapsed_us += poll_interval_us;
    }

    HOLOSCAN_LOG_WARN(
        "Timeout: GPIO {} did not go LOW within {} µs ({} loops)",
        pin, timeout_us, loop_count
    );

    return false;
}


// -------------------- Adcam --------------------

Adcam::Adcam(hololink::DataChannel& hololink_channel,
             uint32_t hololink_i2c_controller_address,
             hololink::Metadata &channel_metadata)
    : hololink_(hololink_channel.hololink()),
      i2c_(hololink_->get_i2c(hololink::CAM_I2C_BUS)), //hololink_->get_i2c(hololink_channel.enumeration_metadata().get<int64_t>("i2c_bus").value())),
      expander0_(hololink_, /*bus*/ hololink::CAM_I2C_BUS, EXPANDER_0_I2C_BUS_ADDRESS),
      expander1_(hololink_, /*bus*/ hololink::CAM_I2C_BUS, EXPANDER_1_I2C_BUS_ADDRESS),
      pf_gpio_(hololink_, channel_metadata, 0 /*pin*/) {
  std::cout << "Initializing ADCAM constructor. 0" << std::endl;
  if (!hololink_) throw std::runtime_error("Adcam: hololink is null");
  //i2c_ = hololink_->get_i2c(hololink_i2c_controller_address);
  if (!i2c_) throw std::runtime_error("Adcam: i2c is null");
  test_ = 1;
}

std::vector<uint8_t> Adcam::get_version() {
  HOLOSCAN_LOG_DEBUG("Fetching Chip version");
  uint16_t reg[] = {1, 0x0112};
  auto resp = set_register16_response(reg, 2);
  HOLOSCAN_LOG_DEBUG("Response = {} bytes", resp.size());

  uint16_t reg1[] = {1, 0x0020};
  resp = set_register16_response(reg1, 2);
  HOLOSCAN_LOG_DEBUG("Status Response = {} bytes", resp.size());
  return resp;
}

std::vector<uint16_t> Adcam::bytes_to_uint16_array_be(const std::vector<uint8_t>& data) {
  if (data.size() % 2 != 0) {
    throw std::runtime_error("Data length must be even for 16-bit conversion.");
  }
  std::vector<uint16_t> out;
  out.reserve(data.size() / 2);
  for (size_t i = 0; i < data.size(); i += 2) {
    uint16_t v = (uint16_t(data[i]) << 8) | uint16_t(data[i + 1]);
    out.push_back(v);
  }
  return out;
}

std::vector<uint8_t> Adcam::registers_to_byte_array_be(uint64_t reg_blob) {
  // Python: length = (bit_length + 7)//8 ; minimum 2 ; even length.
  int bitlen = 0;
  if (reg_blob != 0) bitlen = 64 - __builtin_clzll(reg_blob);
  int length = (bitlen + 7) / 8;
  if (length == 0) length = 2;
  if (length % 2 != 0) length += 1;

  std::vector<uint8_t> out(length);
  for (int i = 0; i < length; i++) {
    int shift = (length - 1 - i) * 8;
    out[i] = uint8_t((reg_blob >> shift) & 0xFF);
  }
  return out;
}

std::vector<uint16_t> Adcam::format_registers_be(uint64_t reg_blob) {
  return bytes_to_uint16_array_be(registers_to_byte_array_be(reg_blob));
}

void Adcam::append_u16_be(std::vector<uint8_t>& out, uint16_t v) {
  out.push_back(uint8_t((v >> 8) & 0xFF));
  out.push_back(uint8_t(v & 0xFF));
}

void Adcam::i2c_write_read(std::shared_ptr<hololink::sensors::Adcam> adcam_inst, const std::vector<uint8_t>& write_bytes,
                           size_t read_byte_count,
                           std::vector<uint8_t>* out_reply) {
  //auto adi_tof_timeout = new hololink::Timeout{30.0, 0.2};
  auto timeout = std::make_shared<hololink::Timeout>(30.0, 0.2);
  if (!i2c_) throw std::runtime_error("i2c is null");
  auto reply = (adcam_inst->i2c_)->i2c_transaction(adcam_inst->ADCAM_I2C_BUS_ADDRESS,
                                     write_bytes,
                                     read_byte_count,
                                     timeout);
  if (out_reply) *out_reply = std::move(reply);
}

void Adcam::i2c_write_read(const std::vector<uint8_t>& write_bytes,
                           size_t read_byte_count,
                           std::vector<uint8_t>* out_reply) {
  //auto adi_tof_timeout = new hololink::Timeout{30.0, 0.2};
  auto timeout = std::make_shared<hololink::Timeout>(30.0, 0.2);
  if (!i2c_) throw std::runtime_error("i2c is null");
  auto reply = i2c_->i2c_transaction(ADCAM_I2C_BUS_ADDRESS,
                                     write_bytes,
                                     read_byte_count,
                                     timeout);
  if (out_reply) *out_reply = std::move(reply);
}

void Adcam::set_register(uint16_t reg, uint8_t value) {
  HOLOSCAN_LOG_DEBUG("set_register(reg=0x{:04X}, value=0x{:02X})", reg, value);
  std::vector<uint8_t> write;
  write.reserve(3);
  append_u16_be(write, reg);
  write.push_back(value);

  i2c_write_read(write, 0, nullptr);
}

void Adcam::set_register16_no_response(uint16_t *register_blob) {
  try {
#if 0
    auto u16s = format_registers_be(register_blob);
    HOLOSCAN_LOG_DEBUG("ADCAM REGISTER NORESPREQD = 0x{:llX}", (unsigned long long)*register_blob);

    std::vector<uint8_t> write;
    write.reserve(u16s.size() * 2);
    for (auto v : u16s) append_u16_be(write, v);
#else
    std::vector<uint8_t> write_bytes;
    write_bytes.resize( register_blob[0] * sizeof(uint16_t));
    core::Serializer serializer(write_bytes.data(), write_bytes.size());
    //HOLOSCAN_LOG_DEBUG("ADCAM REGISTER NORESPREQD = 0x{:llX}", (unsigned long long)*register_blob);
    for (size_t i = 1; i <= register_blob[0]; i++) {
      //append_u16_be(write_bytes, register_blob[i]);
      serializer.append_uint16_be(register_blob[i]);
    }

#endif

    HOLOSCAN_LOG_DEBUG("ADCAM REGISTER NORESPREQD = {} total size {}", static_cast<uint8_t>(write_bytes[0]),write_bytes.size() );
    i2c_write_read(write_bytes, 0, nullptr);
  } catch (const std::exception& e) {
    holoscan::log_error("[ERROR] set_register16_no_response failed: {}", e.what());
  }
}

std::vector<uint8_t> Adcam::set_register16_response(std::shared_ptr<hololink::sensors::Adcam> adcam_inst, uint16_t *register_blob, size_t resp_len) {
  std::vector<uint8_t> reply;
  try {
    std::vector<uint8_t> write_bytes;
    write_bytes.resize( register_blob[0] * sizeof(uint16_t));
    core::Serializer serializer(write_bytes.data(), write_bytes.size());

    for (size_t i = 1; i <= register_blob[0]; i++) {
      //append_u16_be(write_bytes, register_blob[i]);
      serializer.append_uint16_be(register_blob[i]);
    }
    HOLOSCAN_LOG_DEBUG("ADCAM REGISTER RESPREQD = {} total size {}", static_cast<uint8_t>(write_bytes[0]),write_bytes.size() );
    adcam_inst->i2c_write_read(adcam_inst, write_bytes, resp_len, &reply);

    // Python did: deserializer.next_uint8() (skip first byte). We keep reply intact.
    HOLOSCAN_LOG_DEBUG("ADCAM REGISTER REPLY bytes={}", reply.size());
    return reply;
  } catch (const std::exception& e) {
    holoscan::log_error("[ERROR] set_register16_response failed: {}", e.what());
    return reply;
  }
}

std::vector<uint8_t> Adcam::set_register16_response(uint16_t *register_blob, size_t resp_len) {
  std::vector<uint8_t> reply;
  try {
    std::vector<uint8_t> write_bytes;
    write_bytes.resize( register_blob[0] * sizeof(uint16_t));
    core::Serializer serializer(write_bytes.data(), write_bytes.size());

    for (size_t i = 1; i <= register_blob[0]; i++) {
      //append_u16_be(write_bytes, register_blob[i]);
      serializer.append_uint16_be(register_blob[i]);
    }
    HOLOSCAN_LOG_DEBUG("ADCAM REGISTER RESPREQD = {} total size {}", static_cast<uint8_t>(write_bytes[0]),write_bytes.size() );
    i2c_write_read(write_bytes, resp_len, &reply);

    // Python did: deserializer.next_uint8() (skip first byte). We keep reply intact.
    HOLOSCAN_LOG_DEBUG("ADCAM REGISTER REPLY bytes={}", reply.size());
    return reply;
  } catch (const std::exception& e) {
    holoscan::log_error("[ERROR] set_register16_response failed: {}", e.what());
    return reply;
  }
}

void Adcam::set_mipi(std::shared_ptr<hololink::sensors::Adcam> adcam_inst) {
  HOLOSCAN_LOG_DEBUG("Setting MIPI speed ADCAM INST {}", width_);
  //uint64_t reg = 0x00310004;  // 1gbps
  //adcam_inst->get_status();
  uint16_t reg0[] = {1, 0x0020};
  HOLOSCAN_LOG_DEBUG("Fetching status ={}", test_);
  auto resp = adcam_inst->set_register16_response(adcam_inst, reg0, 2);
  HOLOSCAN_LOG_DEBUG("Fetching status 2");
  HOLOSCAN_LOG_INFO("Chip Status bytes={} {} {}", resp.size(), resp[0], resp[1]);
  uint16_t reg[] = {2, 0x0031, 0x0004};  // 1gbps
  adcam_inst->set_register16_no_response(reg);

  adcam_inst->get_status();
  HOLOSCAN_LOG_DEBUG("Enabling deskew");
  uint16_t reg1[] = {2, 0x00AB, 0x0001};
  set_register16_no_response(reg1);
}

void Adcam::set_mode() {
  HOLOSCAN_LOG_DEBUG("Setting QMP mode");
  //uint64_t reg = 0xDA06280F;
  uint16_t reg[] = {2, 0xDA06, 0x280F};
  set_register16_no_response(reg);
}

void Adcam::read_nvm_config() {
  HOLOSCAN_LOG_DEBUG("Reading NVM Config");
  std::this_thread::sleep_for(std::chrono::seconds(1));

  //uint64_t reg = 0x00190000;
  uint16_t reg[] = {2, 0x0019, 0x0000};
  set_register16_no_response(reg);

  //reg = 0xAD002C0500000000ULL;  // NOTE: Python uses >64-bit constants. If you truly need 128-bit,
  uint16_t reg1[] = {4, 0xAD00, 0x2C05, 0x0000, 0x0000}; 
  set_register16_no_response(reg1);
                                // you must pass raw bytes. See note below.
  // For now, keep the pattern and log.
  holoscan::log_warn("read_nvm_config: register blobs >64-bit need a byte-array API in C++.");

  HOLOSCAN_LOG_DEBUG("Reading NVM Config done");
}

void Adcam::get_status() {
  HOLOSCAN_LOG_DEBUG("Fetching status");
  uint16_t reg[] = {1, 0x0020};
  HOLOSCAN_LOG_DEBUG("Fetching status 1");
  auto resp = set_register16_response(reg, 2);
  HOLOSCAN_LOG_DEBUG("Fetching status 2");
  HOLOSCAN_LOG_INFO("Chip Status bytes={} {} {}", resp.size(), resp[0], resp[1]);

  uint16_t reg1[] = {1, 0x0038};
  resp = set_register16_response(reg1, 2);
  HOLOSCAN_LOG_DEBUG("0x0038 Status bytes={} {} {}", resp.size(), resp[0], resp[1]);
}

void Adcam::get_only_status() {
  HOLOSCAN_LOG_DEBUG("Fetching status");
  uint16_t reg[] = {1, 0x0020};
  auto resp = set_register16_response(reg, 2);
  HOLOSCAN_LOG_INFO("Chip Status bytes={} status {} {}", resp.size(), resp[0], resp[1]);
}

int Adcam::probe_adcam_adtf3175() {
  uint16_t reg[] = {1, 0x0112};
  HOLOSCAN_LOG_INFO("probe_adcam_adtf3175 status = {}",  test_);
  auto resp = set_register16_response(reg, 2);

  HOLOSCAN_LOG_INFO("probe_adcam_adtf3175 bytes={} ID {} {} status = {}", resp.size(), resp[0], resp[1], test_);
  // Python compared resp[0]==0x59 && resp[1]==0x31
  if ((resp.size() >= 2) && (resp[0] == 0x59) && (resp[1] == 0x31)) return 1;
  return 0;
}

std::vector<uint8_t> Adcam::force_stop_burst_mode() {
  HOLOSCAN_LOG_DEBUG("Forcing burst mode off");
  uint16_t reg[] = {4, 0xAD00, 0x0010, 0x0000, 0x0000};  // >64-bit in Python in some cases
  set_register16_no_response(reg);
  //holoscan::log_warn("force_stop_burst_mode: register blobs >64-bit need a byte-array API in C++.");
  uint16_t reg1[] = {1, 0x0020};
  return set_register16_response(reg1, 2);
}

std::vector<uint8_t> Adcam::get_fw_version() {
  HOLOSCAN_LOG_DEBUG("Fetching version");
  uint16_t reg[] = {1, 0x0112};
  auto resp = set_register16_response(reg, 2);
  HOLOSCAN_LOG_INFO("Chip ID = {}", resp);

  uint16_t reg1[] = {1, 0x0020};
  resp  = set_register16_response(reg1, 2);
  HOLOSCAN_LOG_INFO("Chip status = {}", resp);

  uint16_t reg2[] = {2, 0x0019, 0x0000};
  set_register16_no_response(reg2);

  //Read fw version
  uint16_t reg3[] = {8, 0xAD00, 0x2C05, 0x0000, 0x0000, 0x3100, 0x0000, 0x0100, 0x0000};
  resp = set_register16_response(reg3, 44);
  HOLOSCAN_LOG_INFO("Firmware ID = {}", resp);

  // turn off burst mode
  uint16_t reg4[] = {8, 0xAD00, 0x0010, 0x0000, 0x0000, 0x1000, 0x0000, 0x0100, 0x0000};
  set_register16_no_response(reg4);

  return resp;
}

void Adcam::get_chip_status() { get_status(); }

void Adcam::stream_on() {
  HOLOSCAN_LOG_DEBUG("Setting Clock continuous mode in stream_on");
  uint16_t reg[] = {2, 0x00A9, 0x0001};
  set_register16_no_response(reg);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  HOLOSCAN_LOG_DEBUG("Turning ON Streaming");
  uint16_t reg1[] = {2, 0x00A, 0x00C5};
  set_register16_no_response(reg1);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  get_status();
}

void Adcam::stream_off() {
  HOLOSCAN_LOG_DEBUG("Turning OFF Streaming");
  uint16_t reg[] = {2, 0x00A, 0x00C5};
  set_register16_no_response(reg);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  get_status();
}

void Adcam::get_ChipID() {
  uint16_t reg[] = {1, 0x0112};
  auto resp = set_register16_response(reg, 2);
  HOLOSCAN_LOG_INFO("Chip ID bytes={} ID {} {}", resp.size(), static_cast<uint8_t>(resp[0]), static_cast<uint8_t>(resp[1]));
}

void Adcam::get_Status() { get_status(); }

std::vector<uint8_t> Adcam::get_ClockContinuousMode() {
  uint16_t reg[] = {1, 0x00AA};
  auto resp = set_register16_response(reg, 2);
  HOLOSCAN_LOG_DEBUG("Clock continuous mode bytes={}", resp.size());
  return resp;
}

void Adcam::adcam_reset_power_on()
                                // hololink::Hololink* /*hololink*/,
                                // hololink::Channel* /*hololink_channel*/,
                                // void* /*channel_metadata*/) 
{
  HOLOSCAN_LOG_DEBUG("Resetting ADCAM");
  HOLOSCAN_LOG_INFO("Resetting ADCAM");
  pf_gpio_.configure_reset_low(0);
  //pf_gpio_.configure_reset_high(0);

  expander0_.set_register(0x0, 0x0);
  expander1_.set_register(0x0, 0x0);

  expander0_.set_register(0x02, 0x02);
  HOLOSCAN_LOG_INFO("Check DS1 LED - it should be ON.");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  expander0_.set_register(0x0, 0x0);
  HOLOSCAN_LOG_INFO("DS1 LED turned off");

  expander0_.set_register(0x20, 0x20);
  expander1_.set_register(0x9, 0x9);

  expander0_.set_register(0x20, 0x20);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  expander0_.set_register(0x20, 0x20);
  expander1_.set_register(0x9, 0x9);

  expander0_.set_register(0x21, 0x21);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  expander0_.set_register(0x23, 0x23);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  expander1_.set_register(0x79, 0x79);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  pf_gpio_.configure_reset_high(0);
  //pf_gpio_.configure_reset_low(0);

  HOLOSCAN_LOG_INFO("booting up ADSD, wait for 10 seconds");
  std::this_thread::sleep_for(std::chrono::seconds(10));
}

void Adcam::adcam_hard_reset()
                            //  hololink::Hololink* /*hololink*/,
                            //  hololink::Channel* /*hololink_channel*/,
                            //  void* /*channel_metadata*/) 
{
  HOLOSCAN_LOG_DEBUG("ADCAM - Making Reset LOW ONLY");

  HOLOSCAN_LOG_DEBUG("ADCAM - Making Reset HIGH ONLY");
  pf_gpio_.configure_reset_high(0);

  HOLOSCAN_LOG_INFO("Waiting 10 secs after reset");
  std::this_thread::sleep_for(std::chrono::seconds(10));
}

void Adcam::profile_fpga_perf(uint32_t pin)
                            //  hololink::Hololink* /*hololink*/,
                            //  hololink::Channel* /*hololink_channel*/,
                            //  void* /*channel_metadata*/) 
{
  HOLOSCAN_LOG_DEBUG("ADCAM - Making Reset LOW ONLY");

  HOLOSCAN_LOG_DEBUG("ADCAM - Making Reset HIGH ONLY");
  pf_gpio_.configure_reset_low(6);
  
  HOLOSCAN_LOG_INFO("Waiting 10 secs after reset");
  std::this_thread::sleep_for(std::chrono::seconds(1));

  pf_gpio_.wait_for_low_and_set_high_profile(6);
}

void Adcam::configure_converter(std::shared_ptr<hololink::csi::CsiConverter> converter)
{
    // where do we find the first received byte?
    uint32_t start_byte = converter->receiver_start_byte();
    //uint32_t transmitted_line_bytes = converter->transmitted_line_bytes(pixel_format_, width_); hololink_module.sensors.csi.PixelFormat.RAW_8
    uint32_t transmitted_line_bytes = converter->transmitted_line_bytes(csi::PixelFormat::RAW_8, width_);
    uint32_t received_line_bytes = converter->received_line_bytes(transmitted_line_bytes);
    converter->configure(
        start_byte,
        received_line_bytes,
        width_,
        height_,
        csi::PixelFormat::RAW_8//pixel_format_
    );
}

uint32_t Adcam::get_width()
{
    return width_;
}
uint32_t Adcam::get_height()
{
    return height_;
}


void Adcam::start(void)
{
    // """Setting and checking Clock continuous mode"""
    HOLOSCAN_LOG_DEBUG("Setting Clock continuous mode in stream_on");
    uint16_t reg[] = {2, 0x00A9, 0x0001};
    set_register16_no_response(reg);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
    HOLOSCAN_LOG_INFO("Turning ON Streaming check the timestamp");
    uint16_t reg_stream_mode_on[] = {2, 0x00AD, 0x00C5};
    set_register16_no_response(reg_stream_mode_on);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    get_status();
      //# self.set_register16_response(0x0058, 2) # read FSYNC status, enabled for debug
}

void Adcam::stop(void)
{
    //"""Stop Streaming"""
    HOLOSCAN_LOG_INFO("Turning OFF Streaming check the timestamp");
    //self.set_register16_response(0x0058, 2)  # read FSYNC status
    uint16_t reg_stream_mode_off[] = {2, 0x000C, 0x0002};
    set_register16_no_response(reg_stream_mode_off);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    get_status();
}

} // namespace hololink::sensors
