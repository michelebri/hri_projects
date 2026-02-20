# TIAGo + Booster T1 Unified Docker

A single Docker container with both **TIAGo** (PAL Robotics, Gazebo/ROS2) and **Booster T1** (Webots simulation) environments.

## Requirements

- Docker with NVIDIA GPU support (`nvidia-container-toolkit`)
- X11 display (for Gazebo and Webots GUIs)

## Build

```bash
docker build -t tiago_booster .
```

> First build takes ~30-40 min (downloads and compiles TIAGo workspace).

## Run

```bash
bash start_tiago.sh
```

This starts the container with:
- NVIDIA GPU access
- X11 display forwarding
- `/exchange` folder mounted from host

---

## TIAGo Simulation (Gazebo)

Open terminals inside the container (`docker exec -ti tiago_sim bash`).

### Launch Gazebo simulation

```bash
ros2 launch tiago_gazebo tiago_gazebo.launch.py
```

### Useful topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | All joint positions |
| `/head_front_camera/rgb/image_raw` | `sensor_msgs/Image` | RGB camera |
| `/head_front_camera/depth/image_raw` | `sensor_msgs/Image` | Depth camera |
| `/scan` | `sensor_msgs/LaserScan` | Laser scan |
| `/mobile_base_controller/odom` | `nav_msgs/Odometry` | Odometry |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |

### Middleware

TIAGo uses **CycloneDDS** (set via `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`).

---

## Booster T1 Simulation (Webots)

Requires **3 terminals** inside the container.

### Terminal 1 — Launch Webots

```bash
start-webots
```

Opens Webots with the T1 world (`T1_release.wbt`). Wait until you see:

```
INFO: 'T1_release' extern controller: Waiting for local or remote connection on port 1234
```

### Terminal 2 — Launch the runner (mck controller)

```bash
start-runner
```

Connects `mck` to Webots on port 1234. The robot will initialize.

### Terminal 3 — Control the robot

```bash
loco
```

#### Commands

| Key | Action |
|-----|--------|
| `mp` | Mode: Prepare |
| `mw` | Mode: Walking (stand up) |
| `md` | Mode: Damping |
| `w` | Walk forward |
| `s` | Walk backward |
| `a` | Strafe left |
| `d` | Strafe right |
| `q` | Rotate left |
| `e` | Rotate right |
| `gu` | Get up from ground |
| `ld` | Lie down |
| `hu/hd/hl/hr` | Head up/down/left/right |
| `ho` | Head center |

**Startup sequence:** `mp` → wait → `mw` to stand up.

### ROS2 topics (Booster)

Booster publishes only 2 ROS2 topics (via FastDDS loopback on `127.0.0.1`):

| Topic | Description |
|-------|-------------|
| `/LocoApiTopicReq` | Commands sent to robot |
| `/LocoApiTopicResp` | Responses from robot |

To see them: `RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 topic list`

### Useful aliases

| Alias | Description |
|-------|-------------|
| `start-webots` | Launch Webots with T1 world |
| `start-runner` | Launch mck controller |
| `start-runner-7dof` | Launch mck (7-DOF arms variant) |
| `loco` | Launch loco client (keyboard control) |
| `bmlog` | Show booster-motion log |
| `bmerr` | Show booster-motion error log |

---

## File Structure

```
.
├── Dockerfile
├── start_tiago.sh          # Docker run script
├── README.md
├── README_BOOSTER.md       # Booster T1 detailed notes
└── booster/
    ├── booster_robotics_sdk/       # Booster C++ SDK
    ├── booster_robotics_sdk_ros2/  # Booster ROS2 SDK
    ├── LocoApiPackage/             # ROS2 messages
    ├── booster_motion/             # mck controller + libs
    ├── configs/                    # system_settings_config.yaml
    ├── webots_updated.zip          # Webots R2023b binary
    ├── webots_simulation.zip       # T1 world files
    ├── booster-runner-webots-full-0.0.11.run
    └── booster-runner-full-webots-7dof_arms-0.0.3.run
```
# hri_projects
