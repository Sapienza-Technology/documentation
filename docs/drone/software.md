# Software

!!! info "Scope of this document"
    This document covers **low-level software** only — RPi system configuration, ROS2 stack, VIO, PX4, and peripheral firmware.
    High-level autonomy and computer vision are covered in separate sections below, but not documented in detail here.

---

## 1. Low-level software

### 1.1 Raspberry Pi — System configuration

- **OS:** Ubuntu 24.04
- **Hostname:** `Drone26-U24`
- **ROS2 distribution:** Jazzy (native, no LXD container)
- **ROS2 sourced automatically** in `~/.bashrc`:

```bash
    source /opt/ros/jazzy/setup.bash
```

- **Workspace:** `~/ros2_ws`
- **Workspace sourced automatically** in `~/.bashrc`:

```bash
    source ~/ros2_ws/install/setup.bash
```

#### Network configuration

| Interface | Mode        | IP / Notes                        |
|-----------|-------------|-----------------------------------|
| `eth0`    | Static      | `10.10.10.10` / `255.255.255.0`   |
| `wlan0`   | DHCP        | Connects to configured WiFi networks |

Static IP configured via NetworkManager. To add a new WiFi network:

```bash
sudo nmcli dev wifi connect "NETWORK_NAME" password "PASSWORD"
```

#### Startup scripts

Two scripts in `~/` manage the drone session:

```bash
~/start_drone.sh   # starts tmux session with all processes
~/stop_drone.sh    # sends Ctrl+C to all windows and kills session
```

See `general.md` for the full startup procedure.

---

### 1.2 MicroXRCE-DDS Agent

Bridge between PX4 and ROS2. Runs on the RPi, communicates with Pixhawk via UART.

```bash
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
```

!!! warning "Must be started before Pixhawk finishes booting"
    See `general.md` — startup procedure section.

**Installation:**

```bash
# [TO FILL] — document installation steps used
```

---

### 1.3 RealSense D435i driver

**Package:** `realsense2_camera` (ROS2)

Launch command used in `start_drone.sh`:

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=1 \
  rgb_camera.color_profile:=640x480x30 \
  enable_depth:=false
```

- Depth disabled — not used by OpenVINS, saves USB 2.0 bandwidth
- IMU topics unified via `unite_imu_method:=1`

**Topics published:**

| Topic | Content |
|-------|---------|
| `/camera/camera/color/image_raw` | RGB image 640x480 @ 30fps |
| `/camera/camera/imu` | Unified IMU data |

**Installation:**

```bash
# [TO FILL] — document installation steps used
```

---

### 1.4 Camera-IMU calibration (Kalibr)

Calibration was performed with **Kalibr** using an AprilGrid target printed in A3 format.

!!! warning "Recalibrate after any camera or IMU mount change"
    If the RealSense is remounted or its orientation changes, calibration must be redone.

??? danger "Calibration is a f***ing pain in the ass"
    Please do it one time only, be sure about everything else, than calibrate.

#### AprilGrid target parameters

| Parameter | Value |
|-----------|-------|
| Tag size  | `[TO FILL]` m |
| Tag spacing | `[TO FILL]` |
| Rows | `[TO FILL]` |
| Cols | `[TO FILL]` |

#### Calibration procedure

**1 — Record calibration bag**

```bash
ros2 bag record --topics /camera/camera/color/image_raw /camera/camera/imu -o ~/calibration_bag
```

Move the drone slowly in front of the AprilGrid for ~90 seconds, covering all angles.

**2 — Convert bag (mcap → ROS1 format)**

```bash
# [TO FILL] — document conversion steps used
```

**3 — Run Kalibr**

```bash
# [TO FILL] — document Kalibr command used
```

**4 — Output files**

Kalibr produces `camchain.yaml` and `imu.yaml`. Copy the relevant values into the OpenVINS config.

#### Current calibration values

```yaml
# [TO FILL] — paste current camchain.yaml and imu.yaml values here after recalibration
```

---

### 1.5 OpenVINS

Visual-Inertial Odometry system. Fuses RealSense RGB + IMU data to estimate drone position.

**Package:** `ov_msckf`
**Config folder:** `~/ros2_ws/src/open_vins/config/d435i/`

Launch command used in `start_drone.sh`:

```bash
ros2 launch ov_msckf subscribe.launch.py \
  config:=d435i \
  max_cameras:=1 \
  use_stereo:=false
```

**Key config file:** `~/ros2_ws/src/open_vins/config/d435i/estimator_config.yaml`

#### ZUPT parameters

Zero velocity update — keeps position stable when the drone is stationary.

```yaml
try_zupt: true
zupt_chi2_multipler: [TO FILL]
zupt_max_velocity: [TO FILL]
zupt_noise_multiplier: [TO FILL]
zupt_max_disparity: [TO FILL]
zupt_only_at_beginning: false
```

#### Camera-IMU time offset

```yaml
calib_cam_timeoffset: false   # online calibration disabled — use fixed value from Kalibr
# [TO FILL] — add fixed timeoffset value
```

**Topic published:**

| Topic | Content |
|-------|---------|
| `/ov_msckf/odomimu` | Position + orientation estimate |

!!! bug "Known issue — VIO drift"
    OpenVINS currently drifts significantly during flight, especially on low-texture surfaces (grass).
    Position mode is unusable until this is resolved.
    Current config values may be incorrect — recalibrate and retune before relying on VIO for autonomous flight.

**Installation:**

```bash
# [TO FILL] — document build steps used
```

---

### 1.6 VIO Bridge

Custom Python script that subscribes to OpenVINS odometry and publishes it to PX4 via MicroXRCE-DDS, so PX4 EKF2 can use VIO as a position source.

**Location:** `~/ros2_ws/vio_bridge.py`

**Run:**

```bash
source ~/ros2_ws/install/setup.bash && python3 ~/ros2_ws/vio_bridge.py
```

#### How it works

OpenVINS → /ov_msckf/odomimu → vio_bridge.py → /fmu/in/vehicle_visual_odometry → MicroXRCE-DDS → PX4 EKF2

The script:

1. Subscribes to `/ov_msckf/odomimu` (ROS2 `Odometry` message)
2. Converts position and orientation to the PX4 NED frame
3. Publishes to `/fmu/in/vehicle_visual_odometry` (PX4 uXRCE-DDS topic)

---

### 1.7 PX4 parameters

Parameters configured on the Pixhawk 6C for this setup. Set via QGroundControl → Parameters.

#### uXRCE-DDS (ROS2 bridge)

| Parameter | Value | Notes |
|-----------|-------|-------|
| `UXRCE_DDS_CFG` | `TELEM2` | UART bridge to RPi |
| `SER_TEL2_BAUD` | `921600` | Must match MicroXRCE agent |
| `UXRCE_DDS_DOM_ID` | `0` | Default domain |

#### Telemetry (ESP8266)

| Parameter | Value | Notes |
|-----------|-------|-------|
| `SER_TEL1_BAUD` | `57600` | Must match ESP8266 firmware config |
| `MAV_0_MODE` | `Normal` | |
| `MAV_0_RATE` | `[TO FILL]` | |

#### RC / SBUS

| Parameter | Value | Notes |
|-----------|-------|-------|
| `[TO FILL]` | `[TO FILL]` | SBUS configuration |

#### EKF2 — VIO fusion

| Parameter | Value | Notes |
|-----------|-------|-------|
| `[TO FILL]` | `[TO FILL]` | EKF2 VIO parameters |

#### GPS - OFF

| Parameter | Value | Notes |
|-----------|-------|-------|
| `[TO FILL]` | `[TO FILL]` | GPS parameters |

#### ESC / Motors

| Parameter | Value | Notes |
|-----------|-------|-------|
| `[TO FILL]` | `[TO FILL]` | DShot600 configuration |

#### VL53L1X range sensor

| Parameter | Value | Notes |
|-----------|-------|-------|
| `[TO FILL]` | `[TO FILL]` | Range sensor configuration |

---

### 1.8 ESP8266 — MavESP8266 firmware

- **Firmware:** MavESP8266 (MAVLink WiFi bridge)
- **Web interface:** `192.168.4.1` (connect to `TechDrone_Telem` first)
- **Baud rate:** `57600` (must match `SER_TEL1_BAUD` in PX4)
- **UDP port:** `14550`

**Flashing:**

```bash
# [TO FILL] — document flashing procedure
```

---

### 1.9 ELRS firmware

- **Protocol on drone side:** SBUS (output from receiver to Pixhawk)
- **Binding:** via passphrase — already configured, stable
- **Firmware version:** `[TO FILL]`

**Rebinding procedure:**

```bash
# [TO FILL] — document rebinding steps if needed
```

---

## 2. High-level software / Autonomy

!!! info "Not documented here"
    The autonomy stack is developed in **Matlab/Simulink** and runs on the Raspberry Pi.
    It publishes commands via ROS2 / MicroXRCE-DDS topics to PX4.
    Documentation for this layer is the responsibility of the autonomy sub-team.

From the low-level perspective, the autonomy stack is just another ROS2 node publishing to PX4 topics — no special configuration required at this level.

---

## 3. Computer Vision

### 3.1 ArUco detection

!!! warning "Work in progress — not documented yet"
    To be completed by the computer vision sub-team.

### 3.2 Probe detection

!!! warning "Work in progress — not documented yet"
    To be completed by the computer vision sub-team.

---

## TODO

- [ ] Fill in all `[TO FILL]` placeholders — requires drone in front of you
- [ ] Document complete RPi setup from scratch (OS install, ROS2, all packages) — essential for filesystem recovery
- [ ] Fix OpenVINS drift — current config and calibration values are unreliable
- [ ] Recalibrate camera-IMU with Kalibr after fixing camera mount definitively
- [ ] Document full PX4 parameter list (export from QGC and paste here)

---

*Low-level / RPi maintainer: Alessandro Paolantonio* — *High-level autonomy / MATLAB Simulink maintainer: Samuele Passeri*