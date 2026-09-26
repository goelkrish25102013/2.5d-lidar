# DIY 2.5D LiDAR Mapper

A homebrew 3D scanning rig built by rotating a 2D LiDAR (RPLidar C1) on a servo/stepper-driven pan axis, producing 3D point clouds from a sensor that only natively sees in 2D. Runs entirely through ROS2 — wired and wireless (WiFi) modes both supported.

## Quick start

### 1. Hardware

- **RPLidar C1** (Slamtec) — 2D lidar, ~55.6 × 55.6 × 41.3mm, UART interface (3.3V TTL), 460800 baud
- **ESP32** (standard dev board, e.g. ESP32-DevKitC or similar) — reads the lidar over UART, drives the pan motor, hosts WiFi + web control panel
- **Pan actuator** — either:
  - A hobby servo (e.g. MG996R) on a PWM pin — simple, capped at ~0–180°
  - A NEMA17 stepper + DRV8825 driver — full range, smoother, needs current-limit (`Vref`) tuning
- **Power**: the RPLidar C1 needs a solid 5V supply (~230mA typical), ideally separate from logic. A dedicated 5V/2A+ wall adapter is strongly recommended over relying on a laptop USB port, especially once WiFi + stepper current spikes are added.

#### Wiring (direct UART, no USB adapter board)

| RPLidar C1 pin | ESP32 pin |
|---|---|
| TX | any UART RX-capable GPIO |
| RX | any UART TX-capable GPIO (must cross over — lidar TX → ESP32 RX, lidar RX → ESP32 TX) |
| 5V | 5V supply |
| GND | common GND |

A standard ESP32 has three hardware UARTs (UART0/1/2), so `HardwareSerial(2)` is used for the lidar here, leaving UART0 free for USB/debug.

### 2. Flash the ESP32

1. Open `esp32_firmware/stepper_wireless` (or `servo_wireless` if using a servo) in the Arduino IDE.
2. Set your WiFi SSID/password and confirm the STEP/DIR or servo pin numbers match your wiring.
3. Flash it. On boot, the Serial Monitor prints the ESP32's IP address and the web control panel URL.

### 3. Build and run the ROS2 package

Requires ROS2 Humble already installed on Ubuntu 22.04. Everything else (dependencies, the workspace build) is handled by the install script:

```bash
chmod +x install.sh
./install.sh
```

Then, in each new terminal you use this project in:
```bash
source ros2_ws/install/setup.bash   # or setup.zsh

ros2 launch lidar_mapper_ros2 lidar_mapper.launch.py mode:=wireless
# or mode:=wired if the lidar's USB adapter is plugged into this machine directly
```

### 4. Visualize

```bash
rviz2
```
Add a `PointCloud2` display on `/accumulated_map`, and a `LaserScan` display on `/scan` if you want the live raw sweep as well.

### 5. Control the rig

Open the ESP32's IP in a browser for the web control panel — jog the pan axis manually, switch between manual/auto-sweep mode, and adjust sweep start/end/step/speed live.

---

## How it works

The RPLidar C1 spins internally in a **vertical plane** (mounted upright, not in its native horizontal orientation). The servo or stepper pans the whole assembly sideways in **azimuth**, sweeping that vertical slice across a room. Every lidar sample `(angle, distance)` gets tagged with the pan axis's angle at that instant, and converted to 3D:

```
x = r * cos(θ) * cos(ψ)
y = r * cos(θ) * sin(ψ)
z = r * sin(θ)
```
where `θ` is the lidar's own scan angle, `ψ` is the pan axis angle, and `r` is the measured distance.

On the ROS2 side, the package publishes a standard `/scan` (`sensor_msgs/LaserScan`), uses a URDF + `robot_state_publisher` to maintain the `base_link → pan_link → lidar_link` TF chain (so the pan angle is applied automatically via TF rather than by hand), and a `scan_accumulator_node` merges scans over time into `/accumulated_map`, saved to `accumulated_map.npy` on shutdown (cleaned via voxel downsampling + statistical outlier removal, using Open3D internally within that node).

## Repository structure

```
2.5d-lidar/
├── ros2_ws/src/lidar_mapper_ros2/   # ROS2 package (wired + wireless modes)
├── esp32_firmware/
│   ├── servo_wireless/               # servo pan axis, WiFi + web UI
│   └── stepper_wireless/             # stepper pan axis, WiFi + web UI
```

## Multi-station 3D mapping (larger spaces)

For spaces bigger than one pan sweep can cover, register multiple scan positions together:

1. **Pure ICP chain + pose graph optimization** — scan from several overlapping positions (~1-2m apart), register each to its neighbor, and run global pose graph optimization for loop closure. No extra hardware needed.
2. **VIO-assisted** — a phone running ARKit-based pose streaming (e.g. Record3DStream, TeleTool) tags each station's sweep with an averaged pose during that dwell, and all stations are merged into one global cloud.

## Known limitations / lessons learned

- **Settle time matters more than expected**: tagging points with the pan angle *before* the motor has actually finished moving/settling produces smooth, systematic curvature (straight walls become curved) — not random noise. Separate "wait for it to stop" from "now collect data" rather than using one combined delay.
- **Stepper microstepping must match firmware exactly**: if `MICROSTEP` in firmware doesn't match the physical `MS1/MS2/MS3` wiring on the driver, every angle is scaled wrong — again producing smooth distortion, not noise.
- **ESP32 hosting its own WiFi (SoftAP) draws more current than joining an existing network (STA mode)**: if you see repeated brownout reboots (`RTCWDT_BROWN_OUT_RESET`) only in AP mode, either power from a proper wall adapter or fall back to STA mode with a static IP / DHCP reservation for a stable, predictable address.
- **The RPLidar C1 needs a protocol-aware driver** — generic RPLidar libraries built for the A-series often fail on the C1 with a "descriptor length mismatch"; the ESP32 firmware here implements the raw UART protocol directly rather than relying on such a library.

## Possible extensions

- Camera-lidar colorization (pinhole projection of 3D points onto a rigidly-mounted camera's frames)
- Slip ring for true continuous 360° rotation without cable winding
- Integration as a primary SLAM sensor for a ground rover (reorient to native 2D horizontal scanning for use with `slam_toolbox`)

## License

MIT (or your preference — add a LICENSE file before making the repo public if you want this enforced)
