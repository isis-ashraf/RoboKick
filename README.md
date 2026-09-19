# RoboKick

An autonomous humanoid leg system that detects a football via computer vision and computes the joint angles needed to kick it ( built on ROS 2 with an ESP32-driven servo leg ).

## Architecture

```
[Camera] → vision_node (HSV detection) → /ball_position
                                              │
                                              ▼
                                  ik_solver (2-link geometric IK/FK)
                                              │
                                              ▼
                                        /joint_angles
                                              │
                                              ▼
                                    serial_bridge (USB serial)
                                              │
                                              ▼
                                  ESP32 firmware (PCA9685 → 6 servos)
```

1. **`ball_detection`** — a ROS 2 node that opens a camera feed, isolates the ball via HSV color thresholding + contour detection, and estimates its 3D position (lateral offset, height, forward distance) using pinhole camera geometry. Publishes to `/ball_position` and `/ball_detected`, plus a debug image overlay to `/vision/debug`.
2. **`ik_solver`** — geometric inverse kinematics for a 2-link leg (thigh 75mm, shin 79mm) using the law of cosines, with forward-kinematics validation on every solve to catch numerical errors. Converts the solved hip/knee angles into a 6-servo command array and publishes to `/joint_angles`. Also publishes `/joint_states` for RViz visualization.
3. **`robot_description`** — URDF model of the leg (base, thigh, shin, foot links) plus a launch file that brings up `robot_state_publisher`, `joint_state_publisher_gui`, and RViz for visualizing and manually testing joint motion.
4. **`serial_bridge`** — bridges `/joint_angles` over USB serial to the ESP32 using a simple text protocol (`ANGLES:h1,k1,a1,h2,k2,a2\n`).
5. **ESP32 firmware** (`firmware/`) — receives serial commands, drives 6 servos via a PCA9685 PWM driver, interpolates smoothly between positions, runs a scripted walk cycle on boot, and executes kick commands before auto-returning to a neutral stance.

## Tech stack

- **Middleware:** ROS 2 (Jazzy)
- **Computer vision:** OpenCV (HSV thresholding, contour detection)
- **Kinematics:** Custom geometric IK/FK (Python, `math`)
- **Firmware:** Arduino framework on ESP32 (PlatformIO), Adafruit PWM Servo Driver library
- **Comms:** USB serial (115200 baud) between ROS 2 host and ESP32
- **Visualization:** RViz2, `robot_state_publisher`

## Repository structure

```
├── ros2_ws/src/
│   ├── ball_detection/     # HSV-based vision node
│   ├── ik_solver/          # IK/FK solver + serial bridge node
│   └── robot_description/  # URDF + RViz launch
└── firmware/                # ESP32 PlatformIO project
    ├── src/main.cpp
    └── platformio.ini
```

## Setup

### ROS 2 side
```bash
cd ros2_ws
colcon build
source install/setup.bash

# Bring up the robot model in RViz
ros2 launch robot_description robot.launch.py

# Run the vision → IK → serial pipeline (separate terminals)
ros2 run ball_detection vision_node
ros2 run ik_solver ik_node
ros2 run ik_solver serial_bridge
```

### ESP32 firmware
```bash
cd firmware
pio run --target upload   # flash to the ESP32
pio device monitor         # view serial output
```

## Current scope & limitations

- **Left leg only is actively driven from vision/IK.** The pipeline solves IK for one leg from the ball position; the right leg's servo channels are currently hardcoded to neutral (0° offset) in `ik_node.py`. Extending to a full bipedal kick would mean solving IK for both legs and coordinating weight transfer between them.
- **Walking is a fixed, timed sequence**, not closed-loop — the ESP32 runs one scripted gait on boot rather than a continuously balanced walk.
- **Kick execution is open-loop**: once a target is reached, the firmware waits, then returns to neutral — there's no feedback confirming the kick actually contacted the ball.
- **Vision assumes a controlled environment** (consistent lighting, calibrated camera height/FOV) since detection relies on HSV thresholds tuned for a specific ball color.

## Possible next steps

- Mirror the IK solve across both legs for a real bipedal kick
- Replace the HSV threshold with a more robust detector (e.g. a lightweight trained model) for varying lighting/ball types
