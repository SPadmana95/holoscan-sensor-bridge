/****************************************************************************
# Copyright (c) 2024 - Analog Devices Inc. All Rights Reserved.
# This software is proprietary & confidential to Analog Devices, Inc.
# and its licensors.
# *****************************************************************************
# *****************************************************************************/

#ifndef ADSD3500_FLASH_HPP
#define ADSD3500_FLASH_HPP

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#define ADI_DUAL_FW_SLOT_SIZE 0x20000  // 128 KB per slot
#define ADI_CHUNK_HEADER_SIZE 20       // ADI chunk header size in bytes

// Forward declaration — full definition is in adcam_lib.hpp
namespace hololink {
namespace sensors {
class Adcam;
}  // namespace sensors
}  // namespace hololink

class Adsd3500 {
  public:
    bool adsd3500_flash(const std::vector<uint8_t>& file_data,
                        std::shared_ptr<hololink::sensors::Adcam> adcam,
                        bool force = false);

  private:
    // ---- Low-level I2C commands ----
    bool write_payload(uint8_t *payload, uint16_t payload_len);
    bool write_cmd(uint16_t cmd, uint16_t value);
    bool read_cmd(uint16_t cmd, uint16_t *data);
    bool read_burst_cmd(uint8_t *payload, uint16_t payload_len, uint8_t *data);

    bool updateAdsd3500MasterFirmware(uint8_t *fw_data, uint32_t fw_len,
                                      bool force, uint32_t expected_crc);
    bool updateAdsd3500SlaveFirmware(uint8_t *fw_data, uint32_t fw_len,
                                     bool force, uint32_t expected_crc);

    // ---- Members ----
    std::shared_ptr<hololink::sensors::Adcam> adcam_;
    bool force = false;
};

#endif // ADSD3500_FLASH_HPP
