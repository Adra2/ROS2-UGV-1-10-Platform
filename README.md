# autonomous_rc_ws

ROS2 workspace for a 1/10 scale autonomous RC car.
**Platform:** Raspberry Pi 4 (4GB) · Ubuntu 22.04 · ROS2 Humble

---

## Hardware

| Component | Model | Interface |
|---|---|---|
| Compute | Raspberry Pi 5 8GB | — |
| PWM Controller | PCA9685 (Dorhea) | I²C bus 1, addr 0x40 |
| Steering Servo | GOUPRC 20kg Low-Profile | PCA9685 ch 0 |
| Brushless Motor | GoolRC 3650 3100KV | — |
| ESC | GoolRC 60A sensorless | PCA9685 ch 1 |
| RC Receiver | DUMBORC X6FG (gyro) | GPIO 18 PPM |
| GPS | Ublox NEO-6M | UART /dev/serial0 |
| Camera | Luxonis OAK-D Lite | USB3 |
| Battery | 2S LiPo 7.4V 5200mAh | — |
| DC-DC | Buck converter 5V/5A | Powers Pi |

---

## Package map

```
src/
├── pca9685_driver/     Low-level I²C driver + generic PWM node  ← already tested ✓
├── vehicle_interface/  servo_node, esc_node  (use pca9685_driver)
├── sensors/            gps_node, rc_receiver_node
├── perception/         camera_node  (OAK-D Lite / DepthAI)
├── control/            teleop_node, autonomous logic  (TODO)
└── bringup/            launch files + config YAMLs
```

---

## Quick start

### 1. First-time setup

```bash
cd ~/autonomous_rc_ws
rosdep install --from-paths src --ignore-src -r -y
pip3 install smbus2 pyserial pynmea2 pigpio depthai
colcon build --symlink-install
source install/setup.bash
```

### 2. Fix: remove stray build artefacts from source tree
```bash
# These ended up inside vehicle_interface/vehicle_interface/ — remove them:
rm -rf src/vehicle_interface/vehicle_interface/build
rm -rf src/vehicle_interface/vehicle_interface/install
rm -rf src/vehicle_interface/vehicle_interface/log
# Always run colcon from ~/autonomous_rc_ws, never from inside src/
```

### 3. Launch hardware only (servo + ESC)

```bash
ros2 launch bringup robot.launch.py hardware_only:=true
```

### 4. Arm ESC (required before throttle commands)

```bash
ros2 service call /esc/arm std_srvs/srv/Trigger
```

### 5. Test steering from command line

```bash
# Centre
ros2 service call /servo/centre std_srvs/srv/Trigger

# 15° right
ros2 topic pub /cmd/steering_angle std_msgs/msg/Float32 '{data: 15.0}' --once

# 15° left
ros2 topic pub /cmd/steering_angle std_msgs/msg/Float32 '{data: -15.0}' --once

# Direct µs (calibration)
ros2 topic pub /cmd/servo_raw_us std_msgs/msg/Int32 '{data: 1500}' --once
```

### 6. Test throttle

```bash
# 20% forward
ros2 topic pub /cmd/throttle std_msgs/msg/Float32 '{data: 0.2}' --once

# Stop
ros2 topic pub /cmd/throttle std_msgs/msg/Float32 '{data: 0.0}' --once
```

### 7. Full launch

```bash
sudo pigpiod                          # start pigpio daemon (for RC receiver)
ros2 launch bringup robot.launch.py
```

---

## Topic map

```
RC Receiver ──/rc/steering──────────────────────────────┐
             ──/rc/throttle──────────────────────────────┤
             ──/rc/mode (manual/auto)                    │
                                                         ▼
GPS      ──/gps/fix                            control/ (TODO)
Camera   ──/camera/rgb/image_raw                        │
         ──/camera/depth/image_raw                      │
                                                    /cmd/steering_angle
                                                    /cmd/throttle
                                                         │
                                             ┌───────────┴───────────┐
                                             ▼                       ▼
                                        servo_node              esc_node
                                        (ch 0, PCA9685)         (ch 1, PCA9685)
                                             │                       │
                                        Steering Servo          Brushless ESC
```

---

## Calibration notes

### Servo centre trim
If wheels aren't straight when `pulse_ctr_us: 1500`:
```bash
# While servo_node is running, try different values:
ros2 topic pub /cmd/servo_raw_us std_msgs/msg/Int32 '{data: 1480}' --once  # trim left
ros2 topic pub /cmd/servo_raw_us std_msgs/msg/Int32 '{data: 1520}' --once  # trim right
# Then update pulse_ctr_us in config/vehicle_interface.yaml
```

### Servo travel limits
```bash
# Find actual mechanical limits (stop before hitting them):
ros2 topic pub /cmd/servo_raw_us std_msgs/msg/Int32 '{data: 1000}' --once  # start here
# Increase/decrease until you feel resistance. Note the values.
# Then update pulse_min_us / pulse_max_us in vehicle_interface.yaml
```

### ESC calibration (if needed)
Some ESCs require a one-time endpoint calibration:
1. With battery disconnected, connect ESC signal wire to PCA9685 ch 1
2. Send 2000 µs, connect battery → ESC beeps high
3. Quickly send 1000 µs → ESC beeps low
4. Send 1500 µs → ESC beeps armed

---

## Deploy to Raspberry Pi via GitHub

```bash
# On Pi (first time):
git clone https://github.com/YOUR_USER/autonomous_rc_ws.git ~/autonomous_rc_ws
cd ~/autonomous_rc_ws
pip3 install smbus2 pyserial pynmea2 pigpio depthai
colcon build --symlink-install
echo "source ~/autonomous_rc_ws/install/setup.bash" >> ~/.bashrc

# Pull updates:
cd ~/autonomous_rc_ws
git pull
colcon build --symlink-install   # only needed if package.xml or entry_points changed
                                  # with --symlink-install, .py edits take effect immediately
```

---

## .gitignore

```
build/
install/
log/
**/__pycache__/
**/*.pyc
**/*.pyo
.colcon_install_layout
```
