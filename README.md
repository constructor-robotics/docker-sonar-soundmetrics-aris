# Sound Metrics ARIS Explorer 3000 - ROS 2 Docker Driver

A Dockerized ROS 2 (Jazzy) driver for the **Sound Metrics ARIS Explorer 3000** forward-looking imaging sonar. This repository provides everything needed to connect to the sonar over Ethernet, receive acoustic frames, and publish them as ROS 2 image topics -- both in raw polar form and as an optional fan-shaped cartesian visualization.

---

## Table of Contents

1. [Hardware Overview](#hardware-overview)
2. [System Architecture](#system-architecture)
3. [Repository Structure](#repository-structure)
4. [Network & Communication Protocol](#network--communication-protocol)
5. [ROS 2 Packages](#ros-2-packages)
   - [soundmetrics_aris_interfaces](#soundmetrics_aris_interfaces)
   - [soundmetrics_aris_drivers](#soundmetrics_aris_drivers)
6. [Driver Node: soundmetrics_aris3000](#driver-node-soundmetrics_aris3000)
   - [Initialization Sequence](#initialization-sequence)
   - [Command Protocol](#command-protocol)
   - [Frame Reception & Reordering](#frame-reception--reordering)
   - [Published Topics & Services](#published-topics--services)
   - [Parameters](#driver-parameters)
7. [Converter Node: polar_to_cartesian](#converter-node-polar_to_cartesian)
   - [Polar-to-Cartesian Mapping](#polar-to-cartesian-mapping)
   - [Grid Overlay](#grid-overlay)
   - [Published Topics](#converter-published-topics)
   - [Parameters](#converter-parameters)
8. [Docker Setup](#docker-setup)
   - [Dockerfile](#dockerfile)
   - [docker-compose.yml](#docker-composeyml)
   - [Environment Variables (.env)](#environment-variables-env)
   - [DDS Middleware Profiles](#dds-middleware-profiles)
   - [Supervisord](#supervisord)
9. [Getting Started](#getting-started)
   - [Prerequisites](#prerequisites)
   - [Network Configuration](#network-configuration)
   - [Build & Run](#build--run)
   - [Verifying the Connection](#verifying-the-connection)
10. [Configuration Reference](#configuration-reference)
11. [Troubleshooting](#troubleshooting)

---

## Hardware Overview

The **ARIS Explorer 3000** is a high-frequency multibeam imaging sonar manufactured by Sound Metrics (a Darley company). It produces near-video-quality acoustic images underwater, functioning much like an underwater camera that works in zero-visibility conditions.

### Key Specifications (ARIS Explorer 3000)

| Parameter | Value |
|---|---|
| Identification Frequency | 3.0 MHz |
| Identification Range | up to 5 m |
| Detection Frequency | 1.8 MHz |
| Detection Range | up to 15 m |
| Number of Beams | 128 (mode 9) |
| Field of View (horizontal) | 28.8 deg (2 x 14.4 deg) |
| Depth Rating | 300 m |
| Max Cable Length | 150 m |
| Dimensions | 26 x 16 x 14 cm |
| Weight in Air / Water | 5.12 kg / 1.55 kg |
| Power Consumption | 18 W typical |
| Power Supply | 48 Vdc (provided AC/DC adapter: 100-240 Vac input) |
| PC Interface | 100BaseT Wired Ethernet |

### How Imaging Works

The ARIS emits multiple acoustic beams that form a fan-shaped (wedge) volume in the water. Each beam returns intensity samples along its range axis. The raw data is a 2D matrix:
- **Rows** = range bins (samples along each beam, from `window_start` to `window_start + window_length`)
- **Columns** = beam indices (angular positions across the 28.8 deg field of view)

This is referred to as the **polar image**. The optional cartesian converter reprojects this into a fan-shaped image suitable for human viewing.

### Physical Setup

The system consists of:
1. **ARIS Explorer sonar head** -- the underwater transducer unit
2. **15 m ARIS Cable** -- composite cable carrying power, Ethernet, and control signals
3. **Command Module** -- a topside box that provides power regulation, status LEDs, and Ethernet breakout
4. **Power Supply** -- 48 Vdc AC/DC adapter connected to the Command Module
5. **Ethernet Cable** -- standard RJ45 from Command Module to the host PC

Connection chain: `PC <--Ethernet--> Command Module <--ARIS Cable--> Sonar Head`

---

## System Architecture

```
+---------------------+        Ethernet (169.254.x.x)        +------------------+
|   Host PC           | <-----------------------------------> | Command Module   |
|   (Docker container)|   TCP :56888 (commands)               |                  |
|                     |   UDP :56444 (sonar data)             +--------+---------+
|  +---------------+  |                                                |
|  | aris3000 node |--+--- publishes:                          ARIS Cable (15m)
|  | (driver)      |  |     /soundmetrics_aris3000/image/polar/raw     |
|  +-------+-------+  |     /soundmetrics_aris3000/sonar_info  +-------v---------+
|          |          |                                        | ARIS Explorer   |
|  +-------v--------+ |                                        | 3000 (sonar)   |
|  | polar_to_cart.  |--+--- publishes:                        +-----------------+
|  | node (optional) |  |     /soundmetrics_aris3000/image/cartesian_fan/raw
|  +-----------------+  |
+---------------------+
```

Both nodes run inside a single Docker container based on `ros:jazzy-perception`. The driver communicates with the sonar directly over TCP/UDP sockets (not through ROS). Sonar frames are then published as ROS 2 `sensor_msgs/Image` messages.

---

## Repository Structure

```
docker-sonar-soundmetrics-aris/
|-- Dockerfile                  # Container image definition (ros:jazzy-perception base)
|-- docker-compose.yml          # Service definition with volumes, env, network
|-- .env                        # Environment variables for docker-compose
|-- entrypoint.sh               # Builds the colcon workspace on container startup
|-- supervisord.conf            # Optional process manager for auto-launching the ROS launch
|-- dds_profiles/               # DDS middleware configuration files
|   |-- cyclonedds_profile.xml
|   |-- fastdds_profile.xml
|   +-- zenoh_config.json5
+-- files/src/                  # ROS 2 workspace source (bind-mounted into container)
    |-- CMakeLists.txt
    |-- soundmetrics_aris_interfaces/    # Custom message/service definitions
    |   |-- CMakeLists.txt
    |   |-- package.xml
    |   |-- msg/
    |   |   +-- SonarInfo.msg
    |   +-- srv/
    |       +-- SetSonarParams.srv
    +-- soundmetrics_aris_drivers/       # Python ROS 2 driver package
        |-- package.xml
        |-- setup.py
        |-- setup.cfg
        |-- config/
        |   +-- soundmetrics_aris3000__standard.yaml   # Default parameter file
        |-- launch/
        |   +-- soundmetrics_aris3000_standard.launch.py
        +-- soundmetrics_aris_drivers/
            |-- __init__.py
            |-- soundmetrics_aris3000.py     # Main sonar driver node
            +-- polar_to_cartesian.py        # Polar-to-cartesian image converter node
```

---

## Network & Communication Protocol

The ARIS Explorer uses a proprietary binary protocol over standard TCP and UDP sockets. The host PC and sonar communicate on a **link-local** Ethernet subnet (169.254.x.x).

### Port Assignments

| Port | Protocol | Direction | Purpose |
|---|---|---|---|
| 56888 | TCP | Host -> Sonar | Command channel (configuration, keep-alive pings) |
| 56444 | UDP | Sonar -> Host | Frame data stream |
| 56123 | UDP | Sonar -> broadcast | Sonar availability announcements |
| 56555 | TCP | (internal) | Command destination port field in protocol headers |

### Command Packet Structure

Every command is a **68-byte header** (17 x 32-bit unsigned integers, little-endian):

| Field Index | Name | Description |
|---|---|---|
| 0 | Checksum | Sum of bytes [4..67] of the packed header |
| 1 | Magic 1 | `0x81B0F8A8` (2175520024) |
| 2 | Magic 2 | `0xAB19AF18` (2868936984) |
| 3 | Protocol Version | `0x00000100` (256) |
| 4 | Command ID | e.g. `1` = PING, `34` = SET_SONAR_PARAMS |
| 5 | Body Size | Always 0 for commands from this driver |
| 6 | Source IP | Host IP as 32-bit integer |
| 7 | Source Port | 0 |
| 8 | Dest IP | Sonar IP as 32-bit integer |
| 9 | Dest Port | 56555 |
| 10 | Transaction Number | Incrementing counter |
| 11-16 | Parameters | Up to 6 command-specific uint32 values |

Each command is sent **twice** over TCP for reliability (a reverse-engineered behavior from the original protocol).

### Key Commands

| Command | ID | Parameters | Purpose |
|---|---|---|---|
| `PING` | 1 | none | Keep-alive, sent every 3 seconds |
| `P2_SET_TARGET_FRAME_PERIOD_USEC` | 43 | frame_period_usec | Set frame rate |
| `P2_SET_RECEIVER_GAIN` | 11 | gain (IEEE 754 as uint32) | Set receiver gain (0-24 dB) |
| `P2_SET_FREQUENCY` | 23 | 0=low, 1=high | Set operating frequency |
| `P2_SET_FOCUS` | 12 | focus_units (0-1000) | Set acoustic lens focus |
| `P2_SET_PULSE_WIDTH` | 26 | pulse_width_usec | Set transmit pulse width |
| `P2_SET_SONAR_PARAMS` | 34 | mode, start_delay, sample_period, cycle_period, samples_per_beam, 0 | Core acoustic parameters |
| `P2_SET_TRANSMIT_ENABLE` | 24 | 1=on | Enable acoustic transmission |
| `P2_V150_ENABLE` | 25 | 1=on | Enable V150 protocol features |

### Frame Data Reception (UDP)

The sonar sends each frame as a **bundle** of UDP packets, each 1400 bytes (68-byte header + 1332-byte payload):
1. The driver first **synchronizes** by waiting for the last packet of a bundle (identified when `packet_num == total_packets - 1` and `body_size > 0`).
2. Subsequent frames are read as complete bundles of `bundle_size` packets.
3. The **first packet** of each bundle contains a 1024-byte **frame header** embedded after the UDP header, followed by image data. Remaining packets contain only image data.
4. Sample values are unsigned 8-bit integers (0-255 intensity).

### Ping Modes (Beam Configurations)

| Mode | Beams | Pings per Frame | Beams per Ping |
|---|---|---|---|
| 1 | 48 | 3 | 16 |
| 3 | 96 | 6 | 16 |
| 6 | 64 | 4 | 16 |
| **9** | **128** | **8** | **16** |

Mode 9 (128 beams, 8 pings) is the default and provides the highest angular resolution across the 28.8 deg field of view.

### Sample Reordering

The raw data from the sonar arrives in a multiplexed order due to the hardware's 16-channel A/D converter. The `reorder_samples()` method applies a fixed **channel reverse map**:

```
[10, 2, 14, 6, 8, 0, 12, 4, 11, 3, 15, 7, 9, 1, 13, 5]
```

This maps each of the 16 physical A/D channels to the correct beam position. The reordering iterates over pings, samples, and channels to place each intensity value at its correct `(range_bin, beam_index)` position in the output image. The final image is left-right flipped to produce the conventional sonar view orientation.

---

## ROS 2 Packages

### soundmetrics_aris_interfaces

A C++ (ament_cmake) package that defines the custom message and service types used by the driver.

#### SonarInfo.msg

Published alongside every frame with metadata extracted from the sonar's 1024-byte frame header:

```
std_msgs/Header header        # Timestamp + frame_id
uint32  index                 # Frame index
uint64  time                  # Sonar internal timestamp
string  version               # Sonar firmware version
float32 window_start          # Range window start (meters)
float32 window_length         # Range window length (meters)
uint32  receiver_gain         # Gain setting
uint32  focus                 # Focus motor position
float32 compass_heading       # Built-in compass heading
float32 compass_pitch         # Built-in compass pitch
float32 compass_roll          # Built-in compass roll
float32 sample_rate           # ADC sample rate
float32 accel_x/y/z           # Built-in accelerometer readings
uint32  ping_mode             # Active ping mode (1/3/6/9)
bool    frequency_hi          # true = high frequency (3 MHz)
uint32  pulse_width           # Transmit pulse width
uint32  cycle_period          # Ping cycle period (usec)
uint32  sample_period         # Sample period (usec)
bool    transmit_enable       # Transmitter on/off
float32 frame_rate            # Measured frame rate
float32 sound_speed           # Sound velocity (from sonar)
uint32  samples_per_beam      # Number of range bins
uint32  beams                 # Number of beams
uint32  pings_per_frame       # Pings per frame
float32 half_field_of_view    # Half FOV in degrees (14.4)
uint32  salinity              # Water salinity
```

#### SetSonarParams.srv

Service interface for runtime reconfiguration (currently commented out in the driver, but available for future use):

```
# Request
float32 frame_period_sec
float32 gain
bool    frequency_hi
uint32  focus
uint32  pulse_width
float32 window_start
float32 window_length
uint32  samples_per_beam
---
# Response
bool attempted
```

### soundmetrics_aris_drivers

A Python (ament_python) package containing two executable nodes.

---

## Driver Node: soundmetrics_aris3000

**File:** `soundmetrics_aris_drivers/soundmetrics_aris3000.py`
**Executable:** `soundmetrics_aris3000`

This is the core driver that communicates directly with the ARIS sonar hardware.

### Initialization Sequence

1. **Load parameters** from the ROS parameter server (populated from the YAML config)
2. **Verify network interface** exists on the host (via `ifconfig`)
3. **Open TCP connection** to the sonar at `sonar_ip:56888`
4. **Send initial configuration** -- frame rate, gain, frequency, focus, pulse width, sonar params, transmit enable
5. **Start ping thread** -- a daemon thread that sends PING commands every 3 seconds to maintain the TCP connection
6. **Open UDP socket** on port 56444 to receive frame data
7. **Create ROS publishers** for polar image, compressed image, and sonar info
8. **Enter main loop** -- calls `read_sonar_image()` continuously, blocking on UDP socket reads

### Command Protocol

The `create_command()` method assembles the 68-byte header, filling in magic numbers, protocol version, source/destination IPs, and a monotonically incrementing transaction number. The `compute_checksum()` static method calculates the checksum over bytes 4-67 and writes it into byte 0.

### Frame Reception & Reordering

The `read_sonar_image()` method:
1. On first call (or after re-sync), enters sync mode: reads packets until it finds the last packet of a bundle. This establishes the `bundle_size`.
2. Reads `bundle_size` UDP packets per frame.
3. Parses the 1024-byte frame header from the first packet into a `SonarInfo` message.
4. Concatenates all payload bytes into a flat list.
5. Calls `reorder_samples()` to undo the hardware multiplexing, producing a `(samples_per_beam x beams)` numpy array.
6. Publishes as a `mono8` ROS Image and a compressed image (PNG or JPEG).
7. Publishes the `SonarInfo` message with the same timestamp.

### Acoustic Parameter Calculation

The `send_config()` method converts human-friendly parameters (meters, seconds) into the sonar's internal timing units (microseconds):

```
sample_start_delay = (window_start * 2 / sound_velocity) * 1e6    # round-trip time
sample_period      = (window_length * 2) / (samples_per_beam * sound_velocity) * 1e6
cycle_period       = sample_start_delay + samples_per_beam * sample_period + 360
```

These values are clamped to hardware limits before being sent.

### Published Topics & Services

All topics are under the namespace `/<ns>/` (default: `/soundmetrics_aris3000/`):

| Topic | Type | Description |
|---|---|---|
| `image/polar/raw` | `sensor_msgs/Image` (mono8) | Raw polar sonar image (rows=range, cols=beams) |
| `image/polar/raw/compressed` | `sensor_msgs/CompressedImage` | Compressed polar image (PNG/JPEG) |
| `sonar_info` | `SonarInfo` | Frame metadata from sonar header |

### Driver Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `use_64_bit_os` | bool | `true` | Adds 4-byte offset when parsing the frame header (required on 64-bit systems) |
| `local_network_interface_name` | string | `"enp4s0"` | Host NIC name connected to the sonar |
| `frame_id` | string | `"aris_sonar_optical_frame"` | TF frame ID for published messages |
| `host_ip` | string | `"169.254.7.10"` | IP address of the host NIC on the sonar subnet |
| `sonar_ip` | string | `"169.254.7.147"` | IP address of the ARIS sonar |
| `frame_period_sec` | float | `0.2` | Target frame period in seconds (0.075 - 1.0) |
| `gain` | int | `24` | Receiver gain in dB (0 - 24) |
| `frequency` | int | `1` | 0 = low frequency (1.8 MHz), 1 = high frequency (3.0 MHz) |
| `focus` | int | `364` | Focus motor position (0 - 1000) |
| `pulse_width` | int | `8` | Transmit pulse width in microseconds |
| `ping_mode` | int | `9` | Beam configuration mode (1, 3, 6, or 9) |
| `samples_per_beam` | int | `1024` | Number of range bins per beam |
| `window_start` | float | `1.2` | Range window start in meters |
| `window_length` | float | `2.0` | Range window length in meters |
| `sound_velocity` | float | `1500.0` | Speed of sound in water (m/s) |
| `cartesian_width` | int | `350` | Width of the cartesian output image (pixels) |
| `compressed_format` | string | `"png"` | Compressed image format (`"png"` or `"jpeg"`) |
| `compressed_quality` | int | `80` | JPEG quality (1-100), ignored for PNG |

---

## Converter Node: polar_to_cartesian

**File:** `soundmetrics_aris_drivers/polar_to_cartesian.py`
**Executable:** `polar_to_cartesian`

An optional node that subscribes to the polar image and sonar info, and produces a fan-shaped cartesian visualization with range/angle grid overlay.

### Polar-to-Cartesian Mapping

The converter uses a **precomputed lookup table** approach for efficient reprojection:

1. **Compute output image dimensions** based on sonar geometry:
   - `pixels_per_meter = samples_per_beam / (rmax - rmin)` -- matches the polar image's range resolution
   - `cart_width = 2 * rmax * sin(half_fov) * pixels_per_meter`
   - `cart_height = rmax * pixels_per_meter`

2. **Build coordinate grids** in meters, centered at the sonar origin (bottom-center of the image).

3. **Convert to polar coordinates** for each output pixel: compute range `R` and angle `Theta` from the cartesian (x, y) position.

4. **Map to polar image indices** using nearest-neighbor interpolation:
   - `range_bin = round((R - rmin) / (rmax - rmin) * (samples_per_beam - 1))`
   - `beam_index = round((Theta + half_fov) / (2 * half_fov) * (beams - 1))`

5. **Apply validity mask** -- only pixels within the sonar's range window and angular FOV are filled; everything else remains black.

The lookup tables are **cached** and only recomputed when sonar parameters change (detected via `needs_remapping()`).

### Time Synchronization

The node uses `message_filters.ApproximateTimeSynchronizer` with a 0.4-second slop to pair incoming polar images with their corresponding `SonarInfo` messages, ensuring the correct parameters are applied.

### Grid Overlay

A **pre-rendered** grid overlay is stamped onto each cartesian image:
- **Range rings** at regular intervals (0.25 m, 0.5 m, or 1.0 m depending on range extent), drawn as arc segments within the FOV, labeled in meters
- **Angle lines** at 5 deg or 10 deg intervals radiating from the sonar origin, labeled in degrees
- A 40-pixel padding border is added around the image for labels

### Converter Published Topics

| Topic | Type | Description |
|---|---|---|
| `image/cartesian_fan/raw` | `sensor_msgs/Image` (mono8) | Fan-shaped cartesian image with grid overlay |
| `image/cartesian_fan/raw/compressed` | `sensor_msgs/CompressedImage` | Compressed cartesian image (PNG/JPEG) |

### Converter Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `enable_scaling` | bool | `true` | Enable output image downscaling |
| `scale_factor` | float | `0.5` | Scale factor when scaling is enabled |
| `compressed_format` | string | `"png"` | Compressed output format |
| `compressed_quality` | int | `80` | JPEG quality (1-100) |

---

## Docker Setup

### Dockerfile

Built from `ros:jazzy-perception`, which includes OpenCV, cv_bridge, and image-related ROS packages. Additional packages installed:
- `supervisor` -- process manager for optionally auto-starting the launch
- `ros-jazzy-rmw-cyclonedds-cpp` -- CycloneDDS middleware
- `ros-jazzy-rmw-zenoh-cpp` -- Zenoh middleware
- Standard tools: `vim`, `git`, `iputils-ping`, `net-tools`

The workspace is created at `/home/ros2ws/` and ROS + workspace sourcing is added to `/root/.bashrc`.

### docker-compose.yml

Key configuration:
- **`network_mode: host`** -- required so the container shares the host's network stack and can reach the sonar on the link-local 169.254.x.x subnet
- **Volumes:**
  - `./files/src` is bind-mounted to `/home/ros2ws/src` -- source code changes on the host are reflected in the container without rebuilding the image
  - `./dds_profiles` is mounted read-only to `/dds_profiles`
- **Environment variables** are loaded from `.env` and passed into the container for ROS domain config and DDS selection

### Environment Variables (.env)

```bash
container_name=ros2-sensor-sonar-aris
hostname=sonar-aris
workspace=ros2ws
rosversion=jazzy
ROS_DOMAIN_ID=0
ROS_LOCALHOST_ONLY=0

# DDS backend -- uncomment the one you want:
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp    # Active by default
# RMW_IMPLEMENTATION=rmw_fastrtps_cpp    # FastDDS (ROS 2 default)
# RMW_IMPLEMENTATION=rmw_zenoh_cpp       # Zenoh (requires zenoh router)

# DDS profile paths (all mounted; only the active one is used)
FASTDDS_DEFAULT_PROFILES_FILE=/dds_profiles/fastdds_profile.xml
CYCLONEDDS_URI=file:///dds_profiles/cyclonedds_profile.xml
RMW_ZENOH_CONFIG_FILE=/dds_profiles/zenoh_config.json5
```

### DDS Middleware Profiles

Three middleware options are pre-configured with factory defaults:
- **CycloneDDS** (default) -- `cyclonedds_profile.xml` -- empty config, uses built-in UDP multicast
- **FastDDS** -- `fastdds_profile.xml` -- empty config, uses built-in UDP multicast
- **Zenoh** -- `zenoh_config.json5` -- client mode connecting to a local router at `tcp/localhost:7447`. Requires starting the Zenoh daemon first: `ros2 run rmw_zenoh_cpp rmw_zenohd`

### Supervisord

The `supervisord.conf` provides an optional way to auto-launch the ROS nodes via a web-controllable process manager:
- **Web UI** available at `http://<host>:9003`
- The `ros2_launch` program is configured with `autostart=false` -- it must be started manually via the supervisord web UI or `supervisorctl`
- Logs are written to `/var/log/supervisor/`

### entrypoint.sh

Runs on container startup:
1. Sources ROS 2 Jazzy setup
2. Runs `colcon build --symlink-install` in the workspace
3. Sources the workspace install
4. Executes the container's CMD (supervisord by default)

Because `--symlink-install` is used and the source is bind-mounted, you can edit Python files on the host and they take effect immediately without rebuilding.

---

## Getting Started

### Prerequisites

- Docker and Docker Compose installed on the host
- The ARIS Explorer 3000 powered on and connected via Ethernet through the Command Module
- A dedicated Ethernet NIC on the host for the sonar connection

### Network Configuration

The ARIS sonar uses **link-local addressing** (169.254.x.x). Configure the host NIC:

```bash
# Identify the NIC connected to the sonar
ip link show

# Assign a static IP in the sonar's subnet
sudo ip addr add 169.254.7.10/16 dev <NIC_NAME>
sudo ip link set <NIC_NAME> up

# Verify connectivity
ping 169.254.7.147
```

Update the `local_network_interface_name` in the YAML config to match your NIC name (e.g., `enp4s0`, `eth0`).

### Build & Run

```bash
# Clone the repository
cd docker-sonar-soundmetrics-aris

# Build the Docker image
docker compose build

# Start the container
docker compose up -d

# Attach to the container for interactive use
docker exec -it ros2-sensor-sonar-aris bash

# Inside the container, launch the driver manually:
ros2 launch soundmetrics_aris_drivers soundmetrics_aris3000_standard.launch.py
```

Alternatively, start the launch via supervisord:
```bash
# From inside the container:
supervisorctl start ros2_launch

# Or from the host, open http://localhost:9003 in a browser
```

### Verifying the Connection

```bash
# In another terminal, attach to the container:
docker exec -it ros2-sensor-sonar-aris bash

# List active topics
ros2 topic list

# Check frame rate on the polar image
ros2 topic hz /soundmetrics_aris3000/image/polar/raw

# View sonar metadata
ros2 topic echo /soundmetrics_aris3000/sonar_info --once

# View the image (requires GUI forwarding)
ros2 run rqt_image_view rqt_image_view
```

---

## Configuration Reference

The main configuration file is `files/src/soundmetrics_aris_drivers/config/soundmetrics_aris3000__standard.yaml`. All parameters are under the `/**` namespace so they apply to both nodes.

### Tuning Guidelines

| Goal | Parameters to Adjust |
|---|---|
| **Increase frame rate** | Decrease `frame_period_sec` (min 0.075s = ~13 Hz). Reduce `samples_per_beam` or `window_length` to shorten cycle time. |
| **Increase range** | Increase `window_start` + `window_length`. Switch to `frequency: 0` (1.8 MHz) for detection mode (up to 15 m). Increase `samples_per_beam` for finer range resolution. |
| **Improve close-range detail** | Use `frequency: 1` (3.0 MHz). Decrease `window_start` to 0.7 m minimum. |
| **Adjust brightness** | Change `gain` (0-24 dB). Start at 12 and increase if image is dark. |
| **Reduce bandwidth** | Set `compressed_format: jpeg` and lower `compressed_quality`. Enable `enable_scaling: true` with a lower `scale_factor`. Disable the cartesian node with `enable_cartesian: false`. |

### Launch Arguments

| Argument | Default | Description |
|---|---|---|
| `ns` | `soundmetrics_aris3000` | ROS namespace for both nodes |
| `enable_cartesian` | from YAML (`false`) | Whether to launch the polar-to-cartesian converter |

Example:
```bash
ros2 launch soundmetrics_aris_drivers soundmetrics_aris3000_standard.launch.py \
    ns:=my_sonar enable_cartesian:=true
```

---

## Troubleshooting

### No data received (UDP timeout warnings)

- Verify the sonar is powered on -- the Command Module LED should be solid blue
- Check that the host NIC has a 169.254.x.x address: `ip addr show <NIC>`
- Ping the sonar: `ping 169.254.7.147`
- Ensure no firewall is blocking UDP port 56444: `sudo ufw allow 56444/udp`
- Verify `local_network_interface_name` in the YAML matches the actual NIC name

### TCP connection refused

- The sonar may not be fully booted yet -- wait for the Command Module LED to be solid blue
- Check that `sonar_ip` and `host_ip` are correct in the YAML config
- Verify TCP port 56888 is reachable: `nc -zv 169.254.7.147 56888`

### "Required local network interface not found"

- The NIC name in the config does not exist on the host. Run `ifconfig` or `ip link` to find the correct name
- When running in Docker with `network_mode: host`, the container sees the host's NICs directly

### Image appears garbled or all black

- Check `ping_mode` -- must be 1, 3, 6, or 9. Invalid modes trigger re-sync
- Verify `use_64_bit_os` is `true` on 64-bit systems (adds a 4-byte offset in frame header parsing)
- Ensure the sonar is submerged or has an acoustic target -- the sonar needs water to produce meaningful images

### Command Module LED Status Codes

| LED Pattern | Meaning | Action |
|---|---|---|
| Solid blue | Normal operation | -- |
| 1 red blink | No messages from ARIS | Cycle power to ARIS |
| 2 red blinks | No link with ARIS | Check ARIS cable connection |
| 3 red blinks | No Ethernet to PC | Check Ethernet cable from CM to PC |
| 4 red blinks (power off) | Temperature > Max | Let unit cool down |
| 5 red blinks (power off) | Breaker Fault | Disconnect/reconnect sonar cable, cycle CM |
| 6 red blinks (power off) | Power > Max | Cycle power to ARIS |
| 7 red blinks (power off) | Voltage > Max | Replace external power supply (must be 48V) |

### High latency or low frame rate

- Reduce `samples_per_beam` (e.g., 512 instead of 1024)
- Reduce `window_length`
- Set `compressed_format: jpeg` to reduce publish overhead
- Disable the cartesian node if not needed

---

## License

MIT License -- Copyright (c) 2025 Robotics Group, Constructor University.

## Maintainer

Robotics Group, Constructor University -- eecs-robotics@lists.jacobs-university.de
