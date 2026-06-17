# ADI ToF Camera Player (`adcam_player`)

A Holoscan-based application for capturing, processing, and visualizing depth
data from an ADI ADTF3175 Time-of-Flight camera connected via the Holoscan
Sensor Bridge (HSB).

---

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Build](#build)
- [Usage](#usage)
- [Command-Line Options](#command-line-options)
- [Execution Flow](#execution-flow)
- [Operator Graph](#operator-graph)
- [ADTFUnpackOp — Frame Unpacking](#adtfunpackop--frame-unpacking)
- [Firmware Update](#firmware-update)
- [Troubleshooting](#troubleshooting)

---

## Overview

`adcam_player` connects to the HSB over the network, initializes the ADTF3175
ToF sensor via I2C, receives MIPI/CSI frames, unpacks the ADI 5-byte-per-pixel
format, and displays three output planes side-by-side using Holoviz:

| Panel | Content |
|-------|---------|
| Left (0 – 33%) | Depth |
| Center (33 – 66%) | Active Brightness |
| Right (66 – 100%) | Confidence |

---

## Prerequisites

- NVIDIA Holoscan SDK
- Holoscan Sensor Bridge connected at `192.168.0.2` (default)
- ADI ADTF3175 sensor module
- CUDA-capable GPU
- (Optional) InfiniBand/ROCE NIC for high-bandwidth reception

---

## Build

### 1. Start the demo container (from the repo root on the devkit)

```bash
cd holoscan-sensor-bridge
sh docker/demo.sh
```

### 2. Inside the container — configure

```bash
export LD_LIBRARY_PATH=/opt/nvidia/holoscan/lib:$LD_LIBRARY_PATH
cmake -S . -B build
```

### 3. Build

```bash
cmake --build build -j$(nproc)
```

### 4. Source directory

```bash
ls examples/aditof/
```

### 5. Output binary

```
./build/examples/aditof/cpp/adcam_player
```

---

## Usage

```bash
./adcam_player [options]
```

### Minimal examples

```bash
# Reset the camera, then capture and display frames
./adcam_player --resetAdcam 1 --capture 1

# Capture only (camera already initialized)
./adcam_player --capture 1

# Headless capture with a 100-frame limit
./adcam_player --capture 1 --headless --frame-limit 100

# Update firmware, then exit
./adcam_player --firmwareUpdate adi_manifest.yaml
```

---

## Command-Line Options

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `--hololink <ip>` | string | `192.168.0.2` | IP address of the HSB |
| `--resetAdcam <0\|1>` | int | `0` | Perform full power-on reset sequence |
| `--resetPin <0-31>` | int | `0` | GPIO pin number used for camera reset |
| `--captureMode <0-6>` | int | `6` | ADCAM capture/mode code |
| `--capture <0\|1>` | int | `0` | Start capture and display pipeline |
| `--firmwareUpdate <file>` | string | — | Path to firmware manifest YAML |
| `--headless` | flag | off | Run Holoviz without a display window |
| `--fullscreen` | flag | off | Run Holoviz in fullscreen mode |
| `--frame-limit <n>` | int | `0` (unlimited) | Stop after N frames |
| `--ibv-name <dev>` | string | auto-detected | InfiniBand device name |
| `--ibv-port <n>` | uint | `1` | InfiniBand port number |
| `--log-level <level>` | string | `info` | Log verbosity: `trace` `debug` `info` `warn` `error` `critical` `off` |
| `-h`, `--help` | flag | — | Print usage |

---

## Execution Flow

The following describes the complete call sequence from `main()`:

### 1. Initialization

```
main()
 ├─ holoscan::set_log_level()         Set Holoscan + HSB log level
 ├─ cuInit() / cuDeviceGet()          Initialize CUDA device
 ├─ cuDevicePrimaryCtxRetain()        Acquire CUDA primary context
 ├─ Enumerator::find_channel()        Discover HSB on the network
 ├─ DataChannel()                     Open the data channel
 └─ Adcam()                           Construct sensor instance (I2C + GPIO)
```

### 2. Reset (if `--resetAdcam 1`)

```
adcam_reset_power_on()
 ├─ configure_reset_low()             Assert GPIO reset pin LOW
 ├─ expander0_.set_register() ×N      Power rail sequencing via I2C expanders
 ├─ configure_reset_high()            Release GPIO reset pin HIGH
 └─ sleep(10s)                        Wait for ADTF3175 boot
```

### 3. Firmware Update (if `--firmwareUpdate <manifest>`)

```
Programmer::fetch_manifest()          Parse manifest YAML
Programmer::check_eula()              Display and accept license terms
Programmer::check_images()            Download/verify firmware images (MD5 + size)
Programmer::program_and_verify_images()
 └─ Adsd3500Flash::adsd3500_flash()   Flash master + slave firmware
       ├─ get_ChipID()                Verify chip responds
       ├─ switch_from_standard_to_burst_mode()
       ├─ get_fw_version_burst_mode() Read current version
       ├─ write firmware pages
       └─ get_fw_version_burst_mode() Verify new version
```
After a successful update the process exits; `--capture` is not required.

### 4. Sensor Probe

```
get_ChipID(GET_MASTER_CHIP_ID_CMD)    Read register 0x0112; log Chip ID bytes
probe_adcam_adtf3175()                Read 0x0112; check ID == {0x59, 0x31}
 └─ returns 1 → prints "ADTF3175 Found"
    returns 0 → prints "ADTF3175 NOT Found" and exits
get_status()                          Read 0x0020 and 0x0038; log chip status
```

### 5. Capture Pipeline (if `--capture 1`)

`compose()` is an override of `holoscan::Application::compose()`. It is **never
called directly** — the Holoscan framework calls it once inside `application->run()` to
build the operator graph before the scheduler starts.

```
main()
 └─ holoscan::make_application<HoloscanApplication>(headless, ..., adcam_inst, frame_limit)
      └─ application->run()
           │
           ├─ HoloscanApplication::compose()   ← called once by the framework
           │     │
           │     ├─ Step 1: make_condition<CountCondition | BooleanCondition>
           │     │           frame limit or run-forever condition
           │     │
           │     ├─ Step 2: make_resource<BlockMemoryPool>("csi_to_bayer_pool")
           │     │           device memory pool (2 blocks, uint16) for CSI→Bayer
           │     │
           │     ├─ Step 3: make_operator<CsiToBayerOp>("csi_to_bayer")
           │     │           allocator=csi_to_bayer_pool, cuda_device_ordinal
           │     │
           │     ├─ Step 4: Camera initialization and configuration
           │     │           probe_adcam_adtf3175()     — confirm sensor reachable
           │     │           configure_converter()      — set CSI frame geometry (width × height)
           │     │           set_mipi()                 — configure MIPI lane speed + deskew
           │     │           set_mode()                 — set QMP capture mode register
           │     │           get_csi_length()           — compute frame_size for receiver
           │     │
           │     ├─ Step 5: make_operator<RoceReceiverOp | LinuxReceiverOp>("receiver")
           │     │           device_start → Adcam::start()
           │     │           device_stop  → Adcam::stop()
           │     │
           │     ├─ Step 6: make_resource<BlockMemoryPool>("ADTF_output_pool")
           │     │           device memory pool (8 blocks, uint16) for ADTFUnpackOp
           │     │
           │     ├─ Step 7: make_operator<ADTFUnpackOp>("ADIToF_data")
           │     │           width=512, height=512, num_planes=3
           │     │
           │     ├─ Step 8: make_operator<HolovizOp>("holoviz")
           │     │           Depth (left) / ActiveBrightness (center) / Conf (right)
           │     │
           │     └─ Step 9: add_flow × 3  — wire the operator graph
           │                 receiver → csi_to_bayer → ADIToF_data → holoviz
           │
           └─ GXF Scheduler starts
                └─ each frame: ADTFUnpackOp::compute() is called automatically
```

Stream start/stop callbacks:
- `device_start` → `Adcam::start()` — enable MIPI clock continuous mode, start streaming
- `device_stop`  → `Adcam::stop()`  — stop streaming

---

## Operator Graph

```
[RoceReceiverOp | LinuxReceiverOp]
         │ output → input
    [CsiToBayerOp]
         │ output → input
     [ADTFUnpackOp]
         │ output → receivers
      [HolovizOp]
    ┌────┴────┬─────────┐
  Depth    ActiveBR   Confidence
```

---

## ADTFUnpackOp — Frame Unpacking

`ADTFUnpackOp` is implemented across two source files and a shared header:

| File | Compiler | Role |
|---|---|---|
| `adcam_unpack_op.hpp` | — | Shared declarations: CUDA kernel launcher prototypes + operator class definition |
| `adcam_unpack_op.cu` | `nvcc` | GPU side — `__global__` CUDA kernels + launch wrapper functions |
| `adcam_unpack_op.cpp` | `g++` | CPU side — Holoscan operator lifecycle + GXF memory management; calls the launch wrappers from the `.cu` |

Both object files are linked together into the `adcam_player` binary.

---

### `adcam_unpack_op.cu` — GPU kernels

Contains all CUDA device code. Each kernel is exposed via a C-callable launcher
declared in the header and called from `adcam_unpack_op.cpp`:

| Kernel | Launcher | Purpose |
|---|---|---|
| `shift_and_cast_kernel` | `shift_and_cast_kernel(..., cudaStream_t)` | Converts `uint16_t` → `uint8_t` by right-shifting 8 bits (CSI buffer arrives as 16-bit words); launcher is a C++ overload of the same name with an extra `cudaStream_t` parameter |
| `unpack_kernel` | `unpack_kernel_launch()` | Splits 5 B/px packed stream → separate `depth[]`, `conf[]`, `ab[]` `uint16_t` arrays — one thread per pixel |
| `jet_kernel` | `jet_kernel_launch()` | Maps `depth[]` → RGB using a 256-entry Jet LUT stored in CUDA `__constant__` memory; depth normalized to 0–4000 mm |
| `grayscale_kernel` | `grayscale_kernel_launch()` | Maps `ab[]` / `conf[]` → grayscale RGB; normalized to 0–4096 |

The Jet LUT is placed in `__constant__` memory for cached broadcast reads across all threads.

#### Kernel purpose details

**`shift_and_cast_kernel`** — Type conversion: `uint16_t` → `uint8_t`

The CSI/MIPI buffer from `CsiToBayerOp` arrives as 16-bit words. The sensor packs its
5-byte/pixel data into those words, but the actual payload byte values fit in 8 bits.
This kernel right-shifts each 16-bit word by 8 bits (`>> 8`) and truncates to `uint8_t`,
recovering the original raw bytes. Without this step the subsequent unpack kernel would
read garbage.

**`unpack_kernel`** — Demultiplex 5-byte pixels into 3 separate planes

One CUDA thread per pixel reads 5 consecutive bytes and extracts:
```
depth[i] = byte[0] | (byte[1] << 8)   → 16-bit distance value
conf[i]  = byte[2] << 8               → 8-bit confidence placed in MSB
ab[i]    = byte[3] | (byte[4] << 8)   → 16-bit active brightness value
```

ADI ToF 5-byte-per-pixel layout:

| Byte | Field | Description |
|------|-------|-------------|
| `[0]` | depth LSB | Lower 8 bits of depth |
| `[1]` | depth MSB | Upper 8 bits of depth → combined: `uint16` distance |
| `[2]` | conf | 8-bit confidence, stored in MSB position (`conf << 8`) |
| `[3]` | ab LSB | Lower 8 bits of active brightness |
| `[4]` | ab MSB | Upper 8 bits of active brightness → combined: `uint16` AB |

```
Byte index:   0      1      2      3      4
             --------------------------------
Data:        D_LSB  D_MSB  CONF   AB_LSB AB_MSB
             --------------------------------

Depth       = [1][0]   (uint16 little-endian)
Confidence  = [2]      (uint8, stored in MSB position)
Brightness  = [4][3]   (uint16 little-endian)
```

Inputs/outputs (all device memory):
- `raw`   — `uint8_t*`  packed input (5 bytes × width × height)
- `depth` — `uint16_t*` unpacked depth plane
- `conf`  — `uint16_t*` unpacked confidence plane
- `ab`    — `uint16_t*` unpacked active brightness plane

Output: three separate `uint16_t` device arrays (`depth[]`, `conf[]`, `ab[]`),
each 512×512 — one value per pixel.

**`jet_kernel`** — Depth → false-color RGB (Jet colormap)

Maps each `uint16_t` depth value to an RGB triplet using a precomputed 256-entry
Jet LUT stored in CUDA `__constant__` memory. Normalization: depth is divided by
4000 mm and scaled to 0–255. Near = blue, mid = green/yellow, far = red. Output:
`uint8_t` RGB image for the **Depth** panel in Holoviz.

**`grayscale_kernel`** — AB / Confidence → grayscale RGB

Maps each `uint16_t` value to a grayscale intensity: divides by 4096 and scales to
0–255, then writes the same value to all three RGB channels. Used twice — once for
**Active Brightness** and once for **Confidence**.

---

### `adcam_unpack_op.cpp` — Holoscan operator logic

Contains the Holoscan lifecycle methods and GXF memory management:

| Method | What it does |
|---|---|
| `setup()` | Registers input/output ports and parameters with the Holoscan framework |
| `start()` | Computes `frame_size_ = width × height` once at pipeline startup |
| `compute()` | Called every frame — full pipeline orchestration (see below) |

**How `compute()` is invoked — framework dispatch**

`compute()` is **never called directly** from `adcam_player.cpp`. The Holoscan
framework calls it automatically on every incoming frame:

```
adcam_player.cpp — HoloscanApplication::compose()
 │
 ├─ make_operator<ADTFUnpackOp>("ADIToF_data", ...)   // register operator
 │
 ├─ add_flow(csi_to_bayer_operator, ADIToF_data,      // wire input
 │           {{"output", "input"}})
 │
 └─ add_flow(ADIToF_data, visualizer,                 // wire output
             {{"output", "receivers"}})

app.run()
 └─ GXF Scheduler
      └─ (each frame, triggered when CsiToBayerOp emits)
           └─ ADTFUnpackOp::compute(op_input, op_output, context)
```

`adcam_player.cpp` only declares *what* to run and *how operators connect* —
the GXF scheduler handles *when* `compute()` is called.

**`compute()` step-by-step per frame:**

```
 1. Receive GXF entity from CsiToBayerOp
 2. Extract CUDA stream from the message
 3. Get raw input tensor (uint16, 5 B/px packed); validate device storage
 4. Get BlockMemoryPool allocator handle
 5. Allocate output GXF tensors: "Depth", "ActiveBrightness", "Conf"  (uint8 RGB, device)
 6. Allocate scratch uint16 tensors: depthraw, confraw, abraw
 7. call shift_and_cast_kernel()   → uint8* raw          [from .cu]
 8. call unpack_kernel_launch()    → depth/conf/ab uint16 [from .cu]
 9. call jet_kernel_launch()       → depth_rgb uint8 RGB  [from .cu]
10. call grayscale_kernel_launch() → ab_rgb, conf_rgb     [from .cu]
11. Emit output GXF entity to HolovizOp
```

---


### Parameters (as configured in `adcam_player.cpp`)

| Parameter | Value | Description |
|---|---|---|
| `num_planes` | `3` | Depth + Active Brightness + Confidence |
| `width` | `512` | Frame width in pixels |
| `height` | `512` | Frame height in pixels |
| `allocator` | `BlockMemoryPool` (8 blocks, device) | GPU memory pool for output tensors |
| `in_tensor_name` | `""` | Unnamed input tensor from `CsiToBayerOp` |
| `out_tensor_name` | `"output"` | Output port name |

### Full data flow — Pre-unpack to Display

```
adcam_unpack_op.hpp        ← shared: kernel launcher prototypes + operator class
        │
        ├── adcam_unpack_op.cu    (nvcc)        adcam_unpack_op.cpp (g++)
        │                                        ADTFUnpackOp::compute()
        │                                          │
        │   [INPUT]                                │  step 1: receive GXF entity
        │   CsiToBayerOp output                    │  step 2: extract CUDA stream
        │   (uint16 words, 5 B/px packed)          │  step 3: get input tensor (uint16)
        │                                          │           validate device storage
        │                                          │  step 4: get BlockMemoryPool handle
        │                                          │  step 5: allocate output GXF tensors
        │                                          │           "Depth" / "ActiveBrightness"
        │                                          │           "Conf"  (uint8 RGB, device)
        │                                          │  step 6: allocate scratch uint16 tensors
        │                                          │           depthraw / confraw / abraw
        │                                          │
        │   shift_and_cast_kernel  ◄───────────────┤  step 7
        │     uint16 → uint8 (>>8)                 │
        │     output: uint8* raw                   │
        │                                          │
        │   unpack_kernel          ◄───────────────┤  step 8
        │     5 B/px → 3 planes                    │
        │     depth[]  uint16                      │
        │     conf[]   uint16                      │
        │     ab[]     uint16                      │
        │                                          │
        │   jet_kernel             ◄───────────────┤  step 9
        │     depth → Jet RGB                      │  GXF tensor "Depth" allocated
        │     (0–4000 mm, near=blue, far=red)      │
        │                                          │
        │   grayscale_kernel ×2    ◄───────────────┤  step 10
        │     ab   → grayscale RGB                 │  GXF tensor "ActiveBrightness"
        │     conf → grayscale RGB                 │  GXF tensor "Conf"
        │                                          │
        │                                          │  step 11: op_output.emit(out_message)
        │                                          │           push 3 tensors downstream
        │
        └── HolovizOp::compute()
              "Depth"            → left viewport   (0.00–0.33)  Jet color
              "ActiveBrightness" → center viewport (0.33–0.66)  Grayscale
              "Conf"             → right viewport  (0.66–1.00)  Grayscale
                    │
              GPU → display window "ADI ToF Player"
```

> All GPU work (unpack + colorization + rendering) stays on the same CUDA stream —
> no host–device synchronization between steps.

### Functions called per step — `ADTFUnpackOp::compute()` (steps 1–11)

All steps run inside `ADTFUnpackOp::compute()` in `adcam_unpack_op.cpp`. Steps 1–6 are CPU-side setup; steps 7–10 launch CUDA kernels; step 11 emits the result downstream.

| Step | Description | Functions called |
|------|-------------|-----------------|
| **1** | Receive GXF entity | `op_input.receive<holoscan::gxf::Entity>("input")` |
| **2** | Extract CUDA stream | `cuda_stream_handler_.from_message()`, `cuda_stream_handler_.get_cuda_stream()` |
| **3** | Get input tensor + validate storage | `entity.get<nvidia::gxf::Tensor>(...)`, `input_tensor->storage_type()`, `input_tensor->size()` |
| **4** | Get allocator handle | `nvidia::gxf::Handle<nvidia::gxf::Allocator>::Create(...)` |
| **5** | Allocate output GXF tensors (`Depth`, `ActiveBrightness`, `Conf`) | `nvidia::gxf::Entity::New()`, `out_message.add<nvidia::gxf::Tensor>(...)`, `tensor->reshape<uint8_t>(...)`, `tensor->data<uint8_t>()` |
| **6** | Allocate scratch `uint16` tensors (`depthraw`, `confraw`, `abraw`) | `scratch_entity.add<nvidia::gxf::Tensor>(...)`, `tensor->reshape<uint16_t>(...)`, `tensor->data<uint16_t>()` |
| **7** | uint16 → uint8 shift (`>>8`) | `shift_and_cast_kernel(raw_u16, raw, size * 5, stream)` |
| **8** | Unpack 5 B/px → depth/conf/ab uint16 planes | `unpack_kernel_launch(raw, depth, conf, ab, width, height, stream)` |
| **9** | depth uint16 → Jet RGB colormap | `jet_kernel_launch(depth, depth_rgb_ptr, size, stream)` |
| **10** | ab/conf uint16 → grayscale RGB | `grayscale_kernel_launch(ab, ab_rgb_ptr, size, stream)`, `grayscale_kernel_launch(conf, conf_rgb_ptr, size, stream)` |
| **11** | Emit output entity with 3 tensors | `op_output.emit(out_message)` |

---

## Firmware Update

### Steps to use the manifest YAML

1. Obtain the firmware binary (`.bin`) for the ADSD3500 sensor.
2. Compute the file size and MD5 checksum:
   ```bash
   wc -c firmware.bin          # file size in bytes
   md5sum firmware.bin         # MD5 hash
   ```
3. Fill in the values in the manifest YAML (`adi_manifest.yaml`) already provided:
   - `filename` — absolute or relative path to the firmware binary (or `url:` for remote fetch)
   - `size` — byte count from step 2
   - `md5` — MD5 hash from step 2
4. Run the updater:
   ```bash
   ./adcam_player --firmwareUpdate adi_manifest.yaml
   ```

The updater will:
1. Parse and validate the manifest
2. Prompt for EULA acceptance (unless `--accept-eula` is set in `Programmer::Args`)
3. Download or read the firmware binary and verify size + MD5
4. Flash master sensor, verify version
5. Flash slave sensor, verify version
6. Exit — power cycle the device before resuming capture

---

## Troubleshooting

| Symptom | Likely Cause | Action |
|---------|-------------|--------|
| `ADTF3175 NOT Found` | Sensor not initialized or powered off | Run with `--resetAdcam 1` |
| `[MASTER] Failed to read Chip ID` | I2C bus not responding | Check HSB network connection and sensor power |
| No frames received | MIPI not streaming | Verify `--captureMode` and `--resetPin` values |
| `Firmware flash failed` | Invalid binary or I2C error | Check manifest MD5/size and sensor power |
| Black/frozen Holoviz window | CUDA or IBV issue | Check `--ibv-name` and CUDA device availability |
