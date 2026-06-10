# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

set(ADITOF_INSTALL_TARGETS)
if(NOT HOLOLINK_BUILD_ONLY_NATIVE)
add_executable(cadcam_player
  cadcam_player.cpp
  )

include(hololink_deps/pybind11)
target_link_libraries(cadcam_player
  PRIVATE
    hololink
    hololink::operators::csi_to_bayer
    hololink::operators::image_processor
    hololink::operators::roce_receiver
    holoscan::core
    holoscan::ops::bayer_demosaic
    holoscan::ops::holoviz
    holoscan::viz::imgui
    holoscan::pybind11
    pybind11::embed
  )

list(APPEND ADITOF_INSTALL_TARGETS cadcam_player)

add_executable(holoviz_samp_disp
  holoviz_samp_disp.cpp
  cadcam_lib.cpp
  cadcam_unpack_op.cpp
  cadcam_unpack_op.cu
  )

include(hololink_deps/pybind11)
target_link_libraries(holoviz_samp_disp
  PRIVATE
    hololink
    hololink::operators::csi_to_bayer
    hololink::operators::image_processor
    hololink::operators::roce_receiver
    hololink::operators::linux_receiver_op
    holoscan::core
    holoscan::ops::bayer_demosaic
    holoscan::ops::holoviz
    holoscan::viz::imgui
    holoscan::pybind11
    pybind11::embed
  )

list(APPEND ADITOF_INSTALL_TARGETS holoviz_samp_disp)


endif() # NOT HOLOLINK_BUILD_ONLY_NATIVE

# Installation of the examples
install(TARGETS ${ADITOF_INSTALL_TARGETS}
        DESTINATION ${CMAKE_INSTALL_BINDIR}
        COMPONENT hololink-examples)
