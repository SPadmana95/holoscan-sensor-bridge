#!/bin/sh

# SPDX-FileCopyrightText: Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#
# See README.md for detailed information.


# This script runs the command given on the command
# line within the hololink demo container.  For example,
#
#   sh docker/demo.sh pytest -s -o log_cli_level=15
#

set -o errexit
set -o xtrace

SCRIPT=`realpath "$0"`
HERE=`dirname "$SCRIPT"`
ROOT=`realpath $HERE/..`
VERSION=`cat $ROOT/VERSION`

NAME=demo

# See if we need to run our container differently.
while [ $# -ge 1 ]
do
case "$1" in
  --name=*)
    NAME="${1#--name=}"
    ;;
  *)
    break
    ;;
esac
shift
done

# enableRawReprocess needs to be set 1 if the frame capture is not \
# done by Argus API and only ISP being used.
# NOTE that we're currently testing with a value of 2 here.
# --- FIX 1: Allow local docker containers to access the host X server ---
if [ -n "$DISPLAY" ]; then
    xhost +local:docker || true
fi

# --- FIX 2: Ensure XDG_RUNTIME_DIR has a safe default if empty ---
if [ -z "$XDG_RUNTIME_DIR" ]; then
    export XDG_RUNTIME_DIR=/tmp/runtime-root
fi

docker run \
    -it \
    --rm \
    --net host \
    --gpus all \
    --runtime=nvidia \
    --shm-size=1gb \
    --privileged \
    --name "$NAME" \
    --ulimit stack=33554432 \
    -v $PWD:$PWD \
    -v $ROOT:$ROOT \
    -v $HOME:$HOME \
    -v /sys/bus/pci/devices:/sys/bus/pci/devices \
    -v /sys/kernel/mm/hugepages:/sys/kernel/mm/hugepages \
    -v /dev:/dev \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /tmp/argus_socket:/tmp/argus_socket \
    -v /sys/devices:/sys/devices \
    -v /var/nvidia/nvcam/settings:/var/nvidia/nvcam/settings \
    -v $XDG_RUNTIME_DIR:$XDG_RUNTIME_DIR \
    -w $PWD \
    -e DISPLAY=${DISPLAY:-:0} \
    -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    -e NVIDIA_DRIVER_CAPABILITIES=graphics,video,compute,utility,display \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e enableRawReprocess=2 \
    hololink-demo:$VERSION \
    $*
