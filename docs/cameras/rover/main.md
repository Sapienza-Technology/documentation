# Rover Cameras Documentation

Guide to start every camera node with commands.

With the jetson, we have an alias for source install/setup.bash, which let us use it simply with s

## 1. ZED 2i
The ZED camera provides depth sensing and spatial tracking, just like the human eye!

### Startup Procedure
1. Open a terminal on the Jetson, navigate to the ZED workspace, and source the environment:
   ```bash
   cd ~/ros2_zed_ws/
   s
   ```
2. Launch the camera node:
   ```bash
   ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
   ```

### Visualization
Keep the Jetson terminal open. On your local PC terminal, run:
```bash
ros2 run rqt_image_view rqt_image_view
```
**Important:** To optimize bandwidth, always select the **Theora** transport type (typically `/zed/zed_node/rgb/image_rect_color/theora`).

---

## 2. Intel RealSense D435i
The RealSense is used for high-accuracy short-range depth sensing, attached on Bracc8.

### Startup Procedure
Open a terminal on the Jetson and execute the following command:
```bash
ros2 launch realsense2_camera rs_launch.py
```

---

## 3. Insta360 X4
The Insta360 provides a 360-degree field of view, processed into five distinct views.

### Camera Startup
1. **Physical Connection:** Ensure the Insta360 is connected via a USB 3.0 cable from the Jetson's USB-A port to the camera's USB-C port.
2. **Device Mode:** Verify the camera is set to **"Webcam Mode"** on its built-in touchscreen.
3. **Hardware Check:** Identify the correct video device index:
   ```bash
   v4l2-ctl --list-devices
   ```
   Look for the Insta360 entry (e.g., `/dev/video2`). If the index is different, replace `video2` in the following command.
4. **Launch:**
   ```bash
   cd ~/insta360_ws
   source install/setup.bash
   ros2 launch insta360_vision insta360.launch.py video_device:=/dev/video2
   ```

### Virtual PTZ Teleop
The Insta360 system allows you to use a "virtual" Pan-Tilt-Zoom camera. This enables you to move the lens digitally to inspect the surroundings.

1. Open a second terminal on the Jetson.
2. Navigate to the workspace and start the teleop node:
   ```bash
   cd ~/insta360_ws
   source install/setup.bash
   ros2 run insta360_vision ptz_teleop
   ```

#### Controls Reference:
*   **WASD / Arrow Keys:** Move the camera view Up, Down, Left, or Right.
*   **1, 2, 3:** Change movement speed presets (Precision, Medium, Fast).
*   **G / F:** Manually increase or decrease the movement step size.
*   **4:** Instant 180-degree turn (U-turn).

---

## Troubleshooting Summary

| Issue | Solution |
| :--- | :--- |
| **Black Screen in rqt** | Verify the camera is in the correct mode and the USB cable supports 3.0 speeds. |
| **Video Device Not Found** | Use `v4l2-ctl --list-devices` to confirm the `/dev/videoX` index. |
| **High Latency** | Ensure you are using compressed topics (Theora for ZED, Compressed for Insta360). |
| **Device Busy** | Run `sudo killall -9 gst-launch-1.0` to clear orphaned GStreamer processes. |
```
