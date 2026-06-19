#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION &
# AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# More details about the chip and eval kit can be found on https://www.analog.com/en/products/adtf3175.html
import logging
import math
import struct
import time

import hololink as hololink_module

import ctypes #Added for FW Update as this needs structure format handling similar to C/C++

################### Firmware update related changes ###################################
# Constants from Adsd3500.cpp for Firmware update
FLASH_PAGE_SIZE = 256
#FLASH_PAGE_SIZE = 128
WRITE_MASTER_FIRMWARE_COMMAND = 0x04
WRITE_SLAVE_FIRMWARE_COMMAND = 0x2A
GET_MASTER_FIRMWARE_COMMAND = 0x01
GET_IMAGER_STATUS_CMD = 0x0020
RESET_ADSD3500_CMD = 0x00240000
GET_MASTER_CHIP_ID_CMD = 0x0112
ADI_STATUS_FIRMWARE_UPDATE = 0x000E
ADI_ROM_CFG_CRC_SEED_VALUE = 0xFFFFFFFF
ADI_ROM_CFG_CRC_POLYNOMIAL = 0x04C11DB7

# Converted from uint32_t const crc32_table[256]. Needed for Firmware update
crc32_table = (
    0, 79764919, 159529838, 222504665, 319059676,
    398814059, 445009330, 507990021, 638119352,
    583659535, 797628118, 726387553, 890018660,
    835552979, 1015980042, 944750013, 1276238704,
    1221641927, 1167319070, 1095957929, 1595256236,
    1540665371, 1452775106, 1381403509, 1780037320,
    1859660671, 1671105958, 1733955601, 2031960084,
    2111593891, 1889500026, 1952343757, 2552477408,
    2632100695, 2443283854, 2506133561, 2334638140,
    2414271883, 2191915858, 2254759653, 3190512472,
    3135915759, 3081330742, 3009969537, 2905550212,
    2850959411, 2762807018, 2691435357, 3560074640,
    3505614887, 3719321342, 3648080713, 3342211916,
    3287746299, 3467911202, 3396681109, 4063920168,
    4143685023, 4223187782, 4286162673, 3779000052,
    3858754371, 3904687514, 3967668269, 881225847,
    809987520, 1023691545, 969234094, 662832811,
    591600412, 771767749, 717299826, 311336399,
    374308984, 453813921, 533576470, 25881363,
    88864420, 134795389, 214552010, 2023205639,
    2086057648, 1897238633, 1976864222, 1804852699,
    1867694188, 1645340341, 1724971778, 1587496639,
    1516133128, 1461550545, 1406951526, 1302016099,
    1230646740, 1142491917, 1087903418, 2896545431,
    2825181984, 2770861561, 2716262478, 3215044683,
    3143675388, 3055782693, 3001194130, 2326604591,
    2389456536, 2200899649, 2280525302, 2578013683,
    2640855108, 2418763421, 2498394922, 3769900519,
    3832873040, 3912640137, 3992402750, 4088425275,
    4151408268, 4197601365, 4277358050, 3334271071,
    3263032808, 3476998961, 3422541446, 3585640067,
    3514407732, 3694837229, 3640369242, 1762451694,
    1842216281, 1619975040, 1682949687, 2047383090,
    2127137669, 1938468188, 2001449195, 1325665622,
    1271206113, 1183200824, 1111960463, 1543535498,
    1489069629, 1434599652, 1363369299, 622672798,
    568075817, 748617968, 677256519, 907627842,
    853037301, 1067152940, 995781531, 51762726,
    131386257, 177728840, 240578815, 269590778,
    349224269, 429104020, 491947555, 4046411278,
    4126034873, 4172115296, 4234965207, 3794477266,
    3874110821, 3953728444, 4016571915, 3609705398,
    3555108353, 3735388376, 3664026991, 3290680682,
    3236090077, 3449943556, 3378572211, 3174993278,
    3120533705, 3032266256, 2961025959, 2923101090,
    2868635157, 2813903052, 2742672763, 2604032198,
    2683796849, 2461293480, 2524268063, 2284983834,
    2364738477, 2175806836, 2238787779, 1569362073,
    1498123566, 1409854455, 1355396672, 1317987909,
    1246755826, 1192025387, 1137557660, 2072149281,
    2135122070, 1912620623, 1992383480, 1753615357,
    1816598090, 1627664531, 1707420964, 295390185,
    358241886, 404320391, 483945776, 43990325,
    106832002, 186451547, 266083308, 932423249,
    861060070, 1041341759, 986742920, 613929101,
    542559546, 756411363, 701822548, 3316196985,
    3244833742, 3425377559, 3370778784, 3601682597,
    3530312978, 3744426955, 3689838204, 3819031489,
    3881883254, 3928223919, 4007849240, 4037393693,
    4100235434, 4180117107, 4259748804, 2310601993,
    2373574846, 2151335527, 2231098320, 2596047829,
    2659030626, 2470359227, 2550115596, 2947551409,
    2876312838, 2788305887, 2733848168, 3165939309,
    3094707162, 3040238851, 2985771188
)

# 1. Define the Packed Struct
# The __attribute__((__packed__)) in C++ is handled by '_pack_ = 1'
#class CmdHeaderStruct(ctypes.LittleEndianStructure):
class CmdHeaderStruct(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("id8", ctypes.c_uint8),             # 0xAD
        ("chunk_size16", ctypes.c_uint16),    # 256 is flash page size
        ("cmd8", ctypes.c_uint8),             # CMD for fw upgrade
        ("total_size_fw32", ctypes.c_uint32), # total size of firmware
        ("header_checksum32", ctypes.c_uint32),# header checksum
        ("crc_of_fw32", ctypes.c_uint32),     # CRC of the Firmware Binary
    ]

# 2. Define the Union
class CmdHeaderUnion(ctypes.Union):
    _fields_ = [
        ("cmd_header_byte", ctypes.c_uint8 * 16), # 16 byte array
        ("fields", CmdHeaderStruct)               # The struct defined above
    ]

# From compute_crc.h
IS_CRC_MIRROR = (1 << 0) #
class CRC_TYPE: #
    CRC_8bit = 8
    CRC_16bit = 16
    CRC_32bit = 32

class CrcOutputUnion(ctypes.Union): #
    _fields_ = [
        ("crc_8bit", ctypes.c_uint8),
        ("crc_16bit", ctypes.c_uint16),
        ("crc_32bit", ctypes.c_uint32),
    ]

class CrcParametersUnion (ctypes.Structure): #
    _fields_ = [
        ("type", ctypes.c_int),             # CRC_TYPE enum
        ("polynomial", CrcOutputUnion),      # Nested Union
        ("initial_crc", CrcOutputUnion),     # Nested Union
        ("crc_compute_flags", ctypes.c_uint32)
    ]

#This is the structure for FW data chunk of 256 bytes
class FwUpdateBinData(ctypes.BigEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("raw_data", ctypes.c_uint8 * 256),             #256 bytes of FW data chunk
    ]

def generate_mirror(value):
    """
    Replicates the generate_mirror logic from compute_crc.c
    Reflects the bits of an 8-bit byte.
    """
    mirror_value = 0
    for i in range(8):
        if (value >> i) & 1:
            mirror_value |= (1 << (7 - i))
    return mirror_value

def compute_crc_python(crc_parameters, data):
    """
    Python equivalent of compute_crc(crc_parameters_t *crc_parameters, ...)
    """
    # Initialize temp_value with the initial_crc (equivalent to memcpy)
    # Accessing the .crc_32bit field from our previously defined ctypes union
    temp_value_crc32 = crc_parameters.initial_crc.crc_32bit
    # Check if we are doing 32-bit CRC
    if crc_parameters.type == 32:  # CRC_32bit = 32
        # In Python, we just use the tuple/list we defined earlier as the table
        # CRC32_TABLE is the tuple you converted in the previous step
        for byte in data:
            if crc_parameters.crc_compute_flags & 1:  # IS_CRC_MIRROR = 1
                # 1. Mirror the input byte
                mirrored_byte = generate_mirror(byte)
                # 2. Calculate the index: mirrored_byte ^ (top byte of current CRC)
                # (temp_value.crc_32bit >> 0x18) & 0xFF extracts the most significant byte
                index = (mirrored_byte ^ (temp_value_crc32 >> 24)) & 0xFF
                # 3. Update CRC: Table[index] ^ (Current CRC shifted left by 8)
                # We MUST mask with 0xFFFFFFFF to keep it 32-bit
                temp_value_crc32 = (crc32_table[index] ^ (temp_value_crc32 << 8)) & 0xFFFFFFFF
    return temp_value_crc32
################### Till here : Firmware update related changes ###################################

class ADCAMEXPANDER:
    EXPANDER_0_I2C_BUS_ADDRESS = 0x68

    def __init__(self, hololink_channel, hololink_i2c_controller_address, expan_addr):
        # Get handles to these controllers but don't actually talk to them yet
        self._hololink = hololink_channel.hololink()
        self._i2c = self._hololink.get_i2c(hololink_i2c_controller_address)
        self._expan_addr = expan_addr

    def set_register(self, register, value, timeout=None):
        logging.debug(
            "set_register(register=%d(0x%X), value=%d(0x%X))"
            % (register, register, value, value)
        )
        write_bytes = bytearray(100)
        serializer = hololink_module.Serializer(write_bytes)
        serializer.append_uint16_be(register)
        serializer.append_uint8(value)
        read_byte_count = 0
        self._i2c.i2c_transaction(
            self._expan_addr,
            write_bytes[: serializer.length()],
            read_byte_count,
            timeout=timeout,
        )
        # print("Expander", self._expan_addr, "byte_count = ", read_byte_count)


class polarfireGpio:

    # configurations to test
    configs = [
        "ALL_OUT_L",  # All pins output low
        "ALL_OUT_H",  # All pins output high
        "ALL_IN",  # All pins inputs
        "ODD_OUT_H",  # Odd pins output high, even pins input - use jumper to short & test
        "EVEN_OUT_H",  # Even pins output high, Odd pins input - use jumper to short & test
    ]

    # dictionary for print beautification
    dir = {0: "output", 1: "input"}

    # def __init__(self, fragment, hololink_channel, gpio, *args, **kwargs):
    def __init__(self, hololink_channel, gpio):
        self._hololink = hololink_channel.hololink()
        self._gpio = gpio
        self.pin = 0
        self.test_config = 0

        # how many pins are supported on the platform running the example
        self._supported_pins_number = self._gpio.get_supported_pin_num()

        logging.info(f"Totel supported GPIOs = {self._supported_pins_number}")

    def setup(self):
        # spec.output("gpio_changed_out")
        # spec.output("test_config_out")

        # set all gpios as output, high - test fast setting via loop
        for i in range(self._supported_pins_number):
            logging.debug(f"GPIO left as is = {i}")

    def pull_reset_low(self, pin):
        self._gpio.set_direction(self.pin, self._gpio.OUT)
        self._gpio.set_value(self.pin, self._gpio.LOW)

    def pull_reset_high(self, pin):
        self._gpio.set_direction(self.pin, self._gpio.OUT)
        self._gpio.set_value(self.pin, self._gpio.HIGH)


# class adtf3175:
class adcam:
    ADCAM_I2C_BUS_ADDRESS = 0x38
    EXPANDER_0_I2C_BUS_ADDRESS = 0x68
    EXPANDER_1_I2C_BUS_ADDRESS = 0x58

    def __init__(
        self, hololink_channel, hololink_i2c_controller_address, channel_metadata
    ):
        # Get handles to these controllers but don't actually talk to them yet
        self._hololink = hololink_channel.hololink()
        self._i2c = self._hololink.get_i2c(hololink_i2c_controller_address)
        self._width = 2560
        self._height = 512
        self._mode = 3  # mode
        self._pixel_format = hololink_module.sensors.csi.PixelFormat.RAW_8
        self._expander0 = ADCAMEXPANDER(
            hololink_channel,
            hololink_module.CAM_I2C_BUS,
            self.EXPANDER_0_I2C_BUS_ADDRESS,
        )
        self._expander1 = ADCAMEXPANDER(
            hololink_channel,
            hololink_module.CAM_I2C_BUS,
            self.EXPANDER_1_I2C_BUS_ADDRESS,
        )
        self._pf_gpio = polarfireGpio(
            hololink_channel, self._hololink.get_gpio(channel_metadata)
        )

    def get_version(self):
        logging.debug("Fatching Chip version")
        REGISTER = 0x0112
        resp = self.set_register16_response(REGISTER, 2)
        logging.debug(f"Response = {resp}")

        # Fetch status
        REGISTER = 0x0020
        resp = self.set_register16_response(REGISTER, 2)
        logging.debug(f"Response = {resp}")

        return resp

    def bytes_to_uint16_array(self, data: bytes):
        """Convert bytes/bytearray to list of 16-bit unsigned integers (big-endian)."""
        if len(data) % 2 != 0:
            raise ValueError("Data length must be even for 16-bit conversion.")
        return list(struct.unpack(f">{len(data)//2}H", data))

    def registers_to_byte_array(self, register):
        """Convert bytes/bytearray to list of 16-bit unsigned integers (big-endian)."""
        length = (register.bit_length() + 7) // 8

        if length == 0:
            length = 2
        if length % 2 != 0:
            length = length + 1

        byte_array = register.to_bytes(length, byteorder="big")
        return byte_array

    def format_registers(self, reg):
        """Convert odd size register to uint16 list (big-endian)."""
        return self.bytes_to_uint16_array(self.registers_to_byte_array(reg))

    def set_mipi(self):
        logging.debug("Setting MIPI speed")
        # REGISTER = 0x00310003 #1.5gbps
        REGISTER = 0x00310004  # 1gbps
        # REGISTER = 0x00310001 #2.5gbps
        self.set_register16_no_response(REGISTER)

        logging.debug("Enabling deskew")
        REGISTER = 0x00AB0001
        self.set_register16_no_response(REGISTER)

    def set_mode(self):
        logging.debug("Setting QMP mode")
        REGISTER = 0xDA06280F
        self.set_register16_no_response(REGISTER)

    def set_mode_for_slave (self):
        logging.debug("Setting mode for Slave pulsatrix")
        print ("Setting mode for Slave pulsatrix")
        REGISTER = 0xDA01280F #Set mode to 0x01
        self.set_register16_no_response(REGISTER)

    def set_slave_threshold (self, RadialValue):
        REGISTER = 0x0027 #Slave Radial value write register
        REGISTER = REGISTER << 16 | RadialValue
        logging.debug(f"Setting radial threshold for Slave pulsatrix to 0x{REGISTER:X}")
        print (f"Setting radial threshold for Slave pulsatrix to 0x{REGISTER:X}")
        self.set_register16_no_response(REGISTER)

    def get_slave_threshold (self):
        logging.debug("Getting radial threshold for Slave pulsatrix")
        print ("Getting radial threshold for Slave pulsatrix")
        REGISTER = 0x0028 #Slave Radial value read register
        resp = self.set_register16_response(REGISTER, 2)
        logging.debug(f"Response = {resp}")
        print (f"Slave reResponse = {resp}")
        return resp

    def softreset (self):
        print("Softresetting the chip")
        logging.info("Softresetting the chip")
        REGISTER = RESET_ADSD3500_CMD
        self.set_register16_no_response(REGISTER)

    def read_nvm_config(self):
        logging.debug("Reading NVM Config")
        time.sleep(1)
        REGISTER = 0x00190000
        self.set_register16_no_response(REGISTER)

        # Read fw version
        REGISTER = 0xAD002C05000000003100000001000000
        resp = self.set_register16_response(REGISTER, 44)
        logging.info(f"Firmware ID = {resp}")

        # Read NVM header
        REGISTER = 0xAD000013000000001300000001000000
        resp = self.set_register16_response(REGISTER, 44)
        logging.debug(f"NVM Header = {resp}")

        # Read fw version
        REGISTER = 0xAD002C05000000003100000001000000
        resp = self.set_register16_response(REGISTER, 44)
        logging.debug(f"Firmware ID = {resp}")

        # turn off burst mode
        REGISTER = 0xAD000010000000001000000000000000
        REGISTER = 0xAD000010000000001000000001000000
        self.set_register16_no_response(REGISTER)
        logging.debug("Reading NVM Config done")

    def get_status(self):
        logging.debug("Fatching status")
        # Get Status
        REGISTER = 0x0020
        resp = self.set_register16_response(REGISTER, 2)
        logging.info(f"Chip Status = {resp}")
        REGISTER = 0x0038
        resp = self.set_register16_response(REGISTER, 2)
        logging.debug(f"0x0038 Status = {resp}")

    def burst_mode_on (self):
        #Turn on burst mode
        REGISTER = 0x00190000
        self.set_register16_no_response(REGISTER)

    def get_second_pulsatrix_ID(self):
        # Get chip ID of 2nd Pulsatrix which is on the slave side
        REGISTER = 0x0116
        print ("Getting Slave Pulsatrix Chip ID")
        resp = self.set_register16_response(REGISTER, 2)
        logging.info(f"Fun: Slave Pulsatrix Chip ID = {resp}")
        print (f"Fun: Slave Pulsatrix Chip ID = {resp}")
        return resp

    def get_generic_resp (self):
        # Get chip ID of 2nd Pulsatrix which is on the slave side
        REGISTER = 0x005A
        print ("Getting generic respons")
        resp = self.set_register16_response(REGISTER, 2)
        logging.info(f"Fun: Generic Response = {resp}")
        print (f"Fun: Generic Response  = {resp}")
        return resp

    def get_only_status(self):
        logging.debug("Fatching status")
        # Get chip ID
        REGISTER = 0x0020
        resp = self.set_register16_response(REGISTER, 2)
        print (f"Chip Status = {resp}")
        logging.info(f"Chip Status = {resp}")
        return resp

    def probe_adcam_adtf3175(self):
        # Get chip ID
        REGISTER = 0x0112
        resp = self.set_register16_response(REGISTER, 2)
        logging.info(f"Chip ID = {resp}")

        if resp[0] == 0x59 and resp[1] == 0x31:
            return 1
        else:
            return 0

    def force_stop_burst_mode(self):
        logging.debug("Forcing burst mode off")

        # turn off burst mode
        REGISTER = 0xAD000010000000001000000001000000
        self.set_register16_no_response(REGISTER)

        return self.get_status()

    def get_fw_version(self):
        logging.debug("Fatching version")

        # Get chip ID
        REGISTER = 0x0112
        resp = self.set_register16_response(REGISTER, 2)
        logging.debug(f"Chip ID = {resp}")

        # Get Status
        REGISTER = 0x0020
        resp = self.set_register16_response(REGISTER, 2)
        logging.debug(f"Chip Status = {resp}")

        # set to burst mode
        REGISTER = 0x00190000
        self.set_register16_no_response(REGISTER)

        # Read fw version
        REGISTER = 0xAD002C05000000003100000001000000
        resp = self.set_register16_response(REGISTER, 44)
        logging.info(f"Firmware ID = {resp}")

        # turn off burst mode
        REGISTER = 0xAD000010000000001000000001000000
        self.set_register16_no_response(REGISTER)

        return resp

    def get_master_fw_version(self):
        logging.info ("Fatching Master version")
        # Read fw version
        REGISTER = 0xAD002C05000000003100000001000000
        resp = self.set_register16_response(REGISTER, 44)
        logging.info(f"Master Firmware ver = {resp}")
        print (f"Master Firmware ver = {resp}")
        return resp

    def get_slave_fw_version(self):
        logging.info ("Fatching Slave version")
        # Read Slave fw version
        REGISTER = 0xAD002C05000000003100000004000000
        #          0xAD002C05000000003100000001000000
        print ("Sending slave FW version read command")
        resp = self.set_register16_response(REGISTER, 44)
        logging.info(f"Slave Firmware Ver = {resp}")
        print (f"Slave Firmware Ver = {resp}")
        return resp

    def get_chip_status(self):
        logging.debug("Fetching status")
        # Get Status
        REGISTER = 0x0020
        resp = self.set_register16_response(REGISTER, 2)
        logging.info(f"Chip Status = {resp}")

        REGISTER = 0x0038
        resp = self.set_register16_response(REGISTER, 2)
        logging.debug(f"Register 0x0038 Status = {resp}")

    def set_register(self, register, value, timeout=None):
        logging.debug(
            "set_register(register=%d(0x%X), value=%d(0x%X))"
            % (register, register, value, value)
        )
        write_bytes = bytearray(100)
        serializer = hololink_module.Serializer(write_bytes)
        serializer.append_uint16_be(register)
        serializer.append_uint8(value)
        read_byte_count = 0
        self._i2c.i2c_transaction(
            self.ADCAM_I2C_BUS_ADDRESS,
            write_bytes[: serializer.length()],
            read_byte_count,
            timeout=timeout,
        )

    def set_register16_no_response(self, register, resp_len=0, timeout=None):
        try:
            write_bytes = bytearray(100)
            uint16_array = self.format_registers(register)
            logging.debug("ADCAM REGISTER NORESPREQD =(0x%X))" % (register))

            serializer = hololink_module.Serializer(write_bytes)
            for i in range(0, len(uint16_array), 1):
                chunk = uint16_array[i]
                # The inner function handles the format string correctly for each chunk
                # serializer.append_uint16_be(struct.unpack('>h', chunk))
                serializer.append_uint16_be(chunk)

            read_byte_count = 0
            self._i2c.i2c_transaction(
                self.ADCAM_I2C_BUS_ADDRESS,
                serializer.data(),
                read_byte_count,
                timeout=timeout,
            )
            # print("ADCAM:", hex(register), "Response", None)
            # logging.debug("set_register16_no_response write =",serializer.data())
        except AttributeError as e:
            logging.info(f"[ERROR] Attribute missing or invalid object used: {e}")
            return None
        except ValueError as e:
            logging.info(f"[ERROR] Value or data format issue: {e}")
            return None
        except OSError as e:
            logging.info(f"[ERROR] I2C communication failed (OS error): {e}")
            return None
        except Exception as e:
            logging.info(f"[ERROR] Unexpected failure during I2C transaction: {e}")
            return None

    def set_register16_response(self, register, resp_len, timeout=None):
        try:
            write_bytes = bytearray(100)
            uint16_array = self.format_registers(register)
            logging.debug("ADCAM REGISTER RESPREQD =(0x%X))" % (register))
            serializer = hololink_module.Serializer(write_bytes)
            for i in range(0, len(uint16_array), 1):
                chunk = uint16_array[i]
                # The inner function handles the format string correctly for each chunk
                serializer.append_uint16_be(chunk)
            read_byte_count = resp_len
            adi_tof_timeout = hololink_module.Timeout(30, retry_s=0.2)
            reply = self._i2c.i2c_transaction(
                self.ADCAM_I2C_BUS_ADDRESS,
                serializer.data(),
                read_byte_count,
                timeout=adi_tof_timeout,
            )
            deserializer = hololink_module.Deserializer(reply)
            deserializer.next_uint8()
            logging.debug(f"ADCAM REGISTER REPLY from Chip = {reply}")
            print("ADCAM:", hex(register), "Response", reply)
            return reply
        except AttributeError as e:
            logging.info(f"[ERROR] Attribute missing or invalid object used: {e}")
            print("ADCAM Attribute Error:", hex(register))
            return reply
        except ValueError as e:
            logging.info(f"[ERROR] Value or data format issue: {e}")
            print("ADCAM ValueError Error:", hex(register))
            return reply
        except OSError as e:
            logging.info(f"[ERROR] I2C communication failed (OS error): {e}")
            print("ADCAM OSError Error:", hex(register))
            return None
        except Exception as e:
            logging.info(f"[ERROR] Unexpected failure during I2C transaction: {e}")
            print("ADCAM exception Error:", hex(register))
            return None

    #Conceptually this funciton is same as def set_register16_no_response(self, register, resp_len=0, timeout=None):
    def write_raw_data (self, data_chunk, data_len, timeout=None):
        try:
            write_bytes = bytearray(1000)
            uint16_array = self.bytes_to_uint16_array(data_chunk)
            serializer = hololink_module.Serializer(write_bytes)
            #print (f"Debug: With response write_raw_data : Len =  {len(uint16_array)}")
            for i in range(0, len(uint16_array), 1):
                chunk = uint16_array[i]
                # The inner function handles the format string correctly for each chunk
                serializer.append_uint16_be(chunk)
            read_byte_count = 0
            #print(f"write_raw_data: Serializer data (hex): {[f'0x{b:02x}' for b in serializer.data()]}")
            # Print data in 16 bytes per line for easier debugging
            data = serializer.data()
            #print("write_raw_data: Serializer data (hex):")
            #for i in range(0, len(data), 16):
            #    line = ' '.join(f'0x{b:02x}' for b in data[i:i+16])
            #    print(f"  [{i:04d}]: {line}")
            self._i2c.i2c_transaction(
                self.ADCAM_I2C_BUS_ADDRESS,
                serializer.data(),
                read_byte_count,
                timeout=timeout
            )
            return True
        except AttributeError as e:
            logging.info(f"[ERROR] Attribute missing or invalid object used: {e}")
            return None
        except ValueError as e:
            logging.info(f"[ERROR] Value or data format issue: {e}")
            return None
        except OSError as e:
            logging.info(f"[ERROR] I2C communication failed (OS error): {e}")
            return None
        except Exception as e:
            logging.info(f"[ERROR] Unexpected failure during I2C transaction: {e}")
            return None

    def read_raw_data (self, data_chunk, read_len, timeout=None):
        try:
            write_dummy_bytes = b'' #This is plain read. So write data is null.
            read_byte_count = read_len
            reply = self._i2c.i2c_transaction(
                self.ADCAM_I2C_BUS_ADDRESS,
                write_dummy_bytes,
                read_byte_count,
                timeout=timeout
            )
            deserializer = hololink_module.Deserializer(reply)
            deserializer.next_uint8()
            logging.info(f"read_raw_data = {reply}")
            print("read_raw_data : Response", reply)
            return reply
        except AttributeError as e:
            logging.info(f"[ERROR] Attribute missing or invalid object used: {e}")
            return None
        except ValueError as e:
            logging.info(f"[ERROR] Value or data format issue: {e}")
            return None
        except OSError as e:
            logging.info(f"[ERROR] I2C communication failed (OS error): {e}")
            return None
        except Exception as e:
            logging.info(f"[ERROR] Unexpected failure during I2C transaction: {e}")
            return None

    def stream_on(self):
        logging.debug("Setting Clock continuous mode in stream_on")
        CONT_MODE = 0x00A90001
        self.set_register16_no_response(CONT_MODE)
        time.sleep(0.2)
        logging.debug("Turning ON Streaming")
        STREAM_MODE = 0x00AD00C5
        self.set_register16_no_response(STREAM_MODE)
        time.sleep(0.2)
        self.get_status()

    def stream_off(self):
        logging.debug("Turning OFF Streaming")
        STREAM_MODE = 0x000C0002
        self.set_register16_no_response(STREAM_MODE)
        time.sleep(0.2)
        self.get_status()

    def get_ChipID(self):
        # Get chip ID
        REGISTER = 0x0112
        resp = self.set_register16_response(REGISTER, 2)
        logging.info(f"Chip ID = {resp}")
        return resp

    def get_Status(self):
        # Get Status
        REGISTER = 0x0020
        resp = self.set_register16_response(REGISTER, 2)
        logging.info(f"Chip Status = {resp}")
        return resp

    def get_ClockContinuousMode(self):
        # Get Status
        REGISTER = 0x00AA
        resp = self.set_register16_response(REGISTER, 2)
        logging.debug(f"Clock continuous mode = {resp}")
        return resp

    #def adcam_reset_power_on(self, hololink, hololink_channel, channel_metadata):
    def adcam_reset_power_on(self):
        logging.debug("Resetting ADCAM")
        self._pf_gpio.pull_reset_low(0)
        self._expander0.set_register(0x0, 0x0)  # Force all the Expandoer 0 bits as 0
        self._expander1.set_register(0x0, 0x0)  # Force all the Expandoer 1 bits as 0

        # O1 (EN_0P8) => 1 //158  Power enable //O7 O6 P5 P4 P3 P2 O1 O0 bits
        # Expander0.get_register(0x02) #This should turn on the DS1 LED ON
        self._expander0.set_register(0x02, 0x02)  # This should turn on the DS1 LED ON
        logging.info("Check DS1 LED - it should be ON. This will be on ")
        # Expander0.get_register(0x02) #This should turn on the DS1 LED ON
        time.sleep(0.2)  # LED ON for 10 sec
        self._expander0.set_register(0x0, 0x0)  # This should turn OFF the DS1 LED
        logging.info("DS1 LED turned off")
        # Checking DONE!

        # P5  (HOST_IO_SEL) => 1 //114 //O7 O6 P5 P4 P3 P2 O1 O0 bits
        self._expander0.set_register(0x20, 0x20)  # E0 = 0x20
        # O8 (HOST_IO_DIR) => 1 //117 //O15 O14 O13 O12 O11 O10 O9 O8 bits
        # O11 (FSYNC_DIR) => 1 //120 //O15 O14 O13 O12 O11 O10 O9 O8 bits
        self._expander1.set_register(0x9, 0x9)  # E1 = 0x09

        # RST to low
        # Add code to make RST GPIO Low

        # O0 (EN_1P8) => 0 //127. Power disable //O7 O6 P5 P4 P3 P2 O1 O0 bits
        # O1 (EN_0P8) => 0 //130. Power disable //O7 O6 P5 P4 P3 P2 O1 O0 bits
        self._expander0.set_register(0x20, 0x20)  # E0 = 0x20 //No change. Remains same
        time.sleep(0.2)  # Pauses execution for 0.2 seconds (200 milliseconds)

        # P3  (I2CM_SET) => 0 //135. Enable SPI for imager to pulsatrix //O7 O6 P5 P4 P3 P2 O1 O0 bits
        # O6 (ISP_BS0) => 0 //138  - Boot strap pins //O7 O6 P5 P4 P3 P2 O1 O0 bits
        # O7 (ISP_BS1) => 0 //141 - Boot strap pins //O7 O6 P5 P4 P3 P2 O1 O0 bits
        self._expander0.set_register(0x20, 0x20)  # E0 = 0x20 //No change. Remains same

        # O9 (ISP_BS4) => 0 //143 - Boot strap pins //O15 O14 O13 O12 O11 O10 O9 O8 bits
        # O10 (ISP_BS5) => 0 //147 - Boot strap pins //O15 O14 O13 O12 O11 O10 O9 O8 bits
        self._expander1.set_register(0x9, 0x9)  # E1 = 0x09 //No change. Remains same

        # O0 (EN_1P8) => 1 //153. Power enable //O7 O6 P5 P4 P3 P2 O1 O0 bits
        self._expander0.set_register(0x21, 0x21)  # E0 = 0x21
        time.sleep(0.2)  # Pauses execution for 0.2 seconds (200 milliseconds)

        # O1 (EN_0P8) => 1 //158  Power enable //O7 O6 P5 P4 P3 P2 O1 O0 bits
        self._expander0.set_register(0x23, 0x23)  # E0 = 0x23
        time.sleep(0.2)  # Pauses execution for 0.2 seconds (200 milliseconds)

        # O14 (EN_VSYS) => 1 //163 //O15 O14 O13 O12 O11 O10 O9 O8 bits
        # O13 (EN_VAUX_LS) => 1 //166 //O15 O14 O13 O12 O11 O10 O9 O8 bits
        # O12 (EN_VAUX) => 1 //169 //O15 O14 O13 O12 O11 O10 O9 O8 bits
        self._expander1.set_register(0x79, 0x79)  # E1 = 0x79
        time.sleep(0.2)  # Pauses execution for 0.2 seconds (200 milliseconds)
        self._pf_gpio.pull_reset_high(0)

        logging.info("booting up ADSD, wait for 10 seconds")
        time.sleep(10)  # Boot-up ADSD3500

    #def adcam_Only_reset(self, hololink, hololink_channel, channel_metadata):
    def adcam_Only_reset(self):
        # pf_gpio = polargpio.polarfireGpio(hololink_channel, hololink.get_gpio(channel_metadata) )
        logging.debug("ADCAM - Making Reset LOW ONLY")
        self._pf_gpio.pull_reset_low(0)

        time.sleep(1)  # Pauses execution for 0.2 seconds (200 milliseconds)
        logging.debug("ADCAM - Making Reset HIGH ONLY")
        self._pf_gpio.pull_reset_high(0)

        logging.info("Waiting 10 secs after reset")
        time.sleep(10)  # Boot-up ADSD3500

    def configure_converter(self, converter):
        # where do we find the first received byte?
        start_byte = converter.receiver_start_byte()
        transmitted_line_bytes = converter.transmitted_line_bytes(
            self._pixel_format, self._width
        )
        received_line_bytes = converter.received_line_bytes(transmitted_line_bytes)
        converter.configure(
            start_byte,
            received_line_bytes,
            self._width,
            self._height,
            self._pixel_format,
        )

    def pixel_format(self):
        return self._pixel_format

    def bayer_format(self):
        return hololink_module.sensors.csi.BayerFormat.RGGB

    def start(self):
        """Setting and checking Clock continuous mode"""
        self.get_status()
        # time.sleep(0.6)
        # mode = self.get_ClockContinuousMode()
        if 0:  # mode[0] == 1:
            logging.debug("Continuous clock mode already enabled")
        else:
            logging.debug("Setting Clock continuous mode")
            CONT_MODE = 0x00A90001
            self.set_register16_no_response(CONT_MODE)
            time.sleep(0.6)
            # Reading the Clock mode for confirmation...
            self.get_ClockContinuousMode()
            time.sleep(0.6)

        """Start Streaming"""
        # self.set_register(0x100, 0x01),
        logging.info(f"Turning ON Streaming TS in sec= {int(time.time())}")
        STREAM_MODE = 0x00AD00C5
        self.set_register16_no_response(STREAM_MODE)
        time.sleep(0.2)
        self.get_status()
        # self.set_register16_response(0x0058, 2) # read FSYNC status, enabled for debug

    def stop(self):
        """Stop Streaming"""
        logging.info(f"Turning OFF Streaming TS in sec= {int(time.time())}")
        self.set_register16_response(0x0058, 2)  # read FSYNC status
        STREAM_MODE = 0x000C0002
        self.set_register16_no_response(STREAM_MODE)
        time.sleep(0.2)
        self.get_status()

    def sendHeader (self, FWData, FWLen, target):
            header = CmdHeaderUnion()
            header.fields.id8 = 0xAD
            header.fields.chunk_size16 = FLASH_PAGE_SIZE
            if target == "master":
                header.fields.cmd8 = WRITE_MASTER_FIRMWARE_COMMAND
            else:
                header.fields.cmd8 = WRITE_SLAVE_FIRMWARE_COMMAND

            header.fields.total_size_fw32 = FWLen
            header.fields.header_checksum32 = 0
            temp_pack = struct.pack(">H B I", header.fields.chunk_size16, header.fields.cmd8, header.fields.total_size_fw32)
            header.fields.header_checksum32 = sum(temp_pack)

            crc_params = CrcParametersUnion()
            crc_params.type = CRC_TYPE.CRC_32bit
            nResidualCRC = ADI_ROM_CFG_CRC_SEED_VALUE
            crc_params.initial_crc.crc_32bit = nResidualCRC
            crc_params.crc_compute_flags = IS_CRC_MIRROR
            res_crc_32bit = compute_crc_python(crc_params, FWData)
            if target == "master":
                print(f"compute_crc_python() for master: returned CRC of: {hex(res_crc_32bit)}")
            else:
                print(f"compute_crc_python() for slave : returned CRC of: {hex(res_crc_32bit)}")
            nResidualCRC = (~res_crc_32bit) & 0xFFFFFFFF
            print(f"Calculated nResidualCRC: {hex(nResidualCRC)}")
            header.fields.crc_of_fw32 = nResidualCRC
            write_header_status = self.write_raw_data(bytes(header), 16)
            return write_header_status

    def perform_FW_update(self, target, bin_buffer, fw_bin_len, stream_buffer, fw_stream_len):
        print("Resetting the ADCAM module before FW update")
        self.adcam_reset_power_on()
        self.get_ChipID()
        if target == "master":
            self.burst_mode_on()
            print("Before FW update : Master FW version is")
            version = self.get_master_fw_version()
            logging.info(f"{version=}")
            if version is None:
                print("Master FW ver = : None (no response)")
            else:
                hex_list = [f"0x{b:02x}" for b in version]
                print(f"Master FW ver = : {hex_list}")
        else:
            '''
            print ("Check if the slave is present or not by setting and getting Redial threshold value")
            self.set_mode_for_slave()
            print ("Reading before setting radial threshold for Slave pulsatrix")
            radial_threshold = self.get_slave_threshold()
            if radial_threshold is not None and len(radial_threshold) >= 2:
                value = (radial_threshold[0] << 8) | radial_threshold[1]
                print(f'Slave radial threshold before setting = 0x{value:X}')
            else:
                print(f'Slave radial threshold before setting = {radial_threshold}')
            time.sleep(0.5)
            self.set_slave_threshold(0x1234)
            time.sleep(0.5)
            radial_threshold = self.get_slave_threshold()
            if radial_threshold is not None and len(radial_threshold) >= 2:
                radial_threshold_value = (radial_threshold[0] << 8) | radial_threshold[1]
                if radial_threshold_value == 0x1234:
                    print("Slave is present and responding correctly.")
                else:
                    print(f"Unexpected radial threshold value: {hex(radial_threshold_value)}")
                    return False
            else:
                print("Failed to read radial threshold from slave. Slave may not be responding.")
                return False
            '''
            print ("Second pulsatrix is present. So setting up burst mode")
            self.burst_mode_on()
            print("Before FW update : Slave FW version is")
            version = self.get_slave_fw_version()
            logging.info(f"{version=}")
            if version is None:
                print("Slave FW ver = : None (no response)")
            else:
                hex_list = [f"0x{b:02x}" for b in version]
                print(f"Slave FW ver = : {hex_list}")

        if target == "master":
            write_header_status = self.sendHeader(bin_buffer, fw_bin_len, target)
        else:
            write_header_status = self.sendHeader(stream_buffer, fw_stream_len, target)
        if not write_header_status:
            print(f"Failed to write hearder. Exiting")
            return False
        if target == "master":
            packets_to_send = math.ceil(fw_bin_len / FLASH_PAGE_SIZE)
            print(f"Writing Firmware packets ({packets_to_send} total)...")
            for i in range(packets_to_send):
                start = i * FLASH_PAGE_SIZE
                end = start + FLASH_PAGE_SIZE
                time.sleep(0.1)
                print(f"Writing Firmware packet {i+1} of {packets_to_send}...")
                chunk = bin_buffer[start:end]
                if len(chunk) < FLASH_PAGE_SIZE:
                    chunk = chunk.ljust(FLASH_PAGE_SIZE, b'\x00')
                if not self.write_raw_data(bytes(chunk), FLASH_PAGE_SIZE):
                    print(f"\nFailed to send packet number {i+1} out of {packets_to_send}!")
                    self.force_stop_burst_mode()
                    return False
        else:
            packets_to_send = math.ceil(fw_stream_len / FLASH_PAGE_SIZE)
            print(f"Writing Firmware packets ({packets_to_send} total)...")
            for i in range(packets_to_send):
                start = i * FLASH_PAGE_SIZE
                end = start + FLASH_PAGE_SIZE
                time.sleep(0.1)
                print(f"Writing Firmware packet {i+1} of {packets_to_send}...")
                chunk = stream_buffer[start:end]
                if len(chunk) < FLASH_PAGE_SIZE:
                    chunk = chunk.ljust(FLASH_PAGE_SIZE, b'\x00')
                if not self.write_raw_data(bytes(chunk), FLASH_PAGE_SIZE):
                    print(f"\nFailed to send packet number {i+1} out of {packets_to_send}!")
                    self.force_stop_burst_mode()
                    return False

        print("Updating FW is completed. Turning off burst mode")
        self.force_stop_burst_mode()
        print("Waiting for 20 secs")
        time.sleep(20)
        fw_update_status = self.get_Status()
        if target == "master":
            if fw_update_status == [0x00, 0x0e] or fw_update_status == [0, 14]:
                print("Master Firmware update Success!!!")
            else:
                print(f"Master Firmware update failed. Status: {fw_update_status}")
        else:
            if fw_update_status == [0x00, 0x27] or fw_update_status == [0, 39]:
                print("Slave Firmware update Success!!!")
            else:
                print(f"Slave Firmware update failed. Status: {fw_update_status}")

        print("Relset pulsatrix")
        self.softreset()
        print("Waiting for 10 secs")
        time.sleep(10)
        print("Reading Chip ID")
        self.get_ChipID()
        print("Reading Status")
        self.get_status()
        self.burst_mode_on()
        if target == "master":
            self.get_master_fw_version()
        else:
            self.get_slave_fw_version()
        return True
