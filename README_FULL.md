# TIAGo + Booster T1 — Simulation Stack

Ambiente di simulazione integrato per robot eterogenei basato su Docker, ROS2, Webots, MuJoCo, Circus e SimBridge.

## Indice

- [Architettura](#architettura)
- [Requisiti](#requisiti)
- [Struttura del progetto](#struttura-del-progetto)
- [Docker — TIAGo e Booster T1](#docker--tiago-e-booster-t1)
- [Circus — Simulatore MuJoCo](#circus--simulatore-mujoco)
- [SimBridge — Ponte ROS2 ↔ Circus](#simbridge--ponte-ros2--circus)
- [Flusso completo](#flusso-completo)

---

## Architettura

Il sistema è composto da tre componenti principali che interagiscono tra loro:

```
┌─────────────────────────────────────────────────────────┐
│  Host                                                   │
│                                                         │
│  ┌──────────┐   Docker API    ┌──────────────────────┐  │
│  │  Circus  │ ──────────────► │  Docker Container    │  │
│  │ (MuJoCo) │                 │  ┌────────────────┐  │  │
│  │          │ ◄── TCP:5555 ── │  │   SimBridge    │  │  │
│  └──────────┘   MessagePack   │  │   (ROS2 node)  │  │  │
│                               │  └────────────────┘  │  │
│                               │  TIAGo / Booster T1  │  │
│                               └──────────────────────┘  │
└─────────────────────────────────────────────────────────┘
```

- **Circus** gestisce la fisica (MuJoCo) e il ciclo di simulazione
- **Docker container** isola il software del robot (ROS2, framework di controllo)
- **SimBridge** traduce tra ROS2 topics e il protocollo TCP di Circus

---

## Requisiti

- Docker con NVIDIA GPU support (`nvidia-container-toolkit`)
- X11 display (per Gazebo e Webots)
- `pixi` (per build di Circus e SimBridge) — [installazione](https://pixi.sh)
- GPU NVIDIA (richiesta da Circus per MuJoCo rendering)

---

## Struttura del progetto

```
tiago_booster_docker/
├── Dockerfile                  # Immagine unificata TIAGo + Booster
├── docker/
│   ├── entrypoint.sh           # Dispatcher (legge $ROBOT_STACK)
│   ├── entrypoint_tiago.sh     # Bootstrap ambiente TIAGo
│   └── entrypoint_booster.sh   # Bootstrap ambiente Booster
├── booster/
│   ├── booster_robotics_sdk/       # Booster C++ SDK
│   ├── booster_robotics_sdk_ros2/  # Booster ROS2 SDK
│   ├── LocoApiPackage/             # Messaggi ROS2 Booster
│   ├── booster_motion/             # Controller mck + librerie
│   ├── camera_supervisor/          # Publisher camera Webots
│   └── configs/
├── exchange/
│   └── T1_release.wbt          # World Webots modificato
├── circus/                     # Simulatore MuJoCo (submodule/cartella)
│   ├── src/
│   ├── pixi.toml
│   └── CMakeLists.txt
└── simbridge/                  # Ponte ROS2 ↔ Circus (submodule/cartella)
    ├── src/
    └── pixi.toml
```

---

## Docker — TIAGo e Booster T1

### Build

```bash
docker build -t tiago_booster .
```

> La prima build richiede ~30-40 minuti (scarica e compila il workspace TIAGo).

### Run

```bash
# TIAGo
docker run -it --rm \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -e ROBOT_STACK=tiago \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  tiago_booster

# Booster T1
docker run -it --rm \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -e ROBOT_STACK=booster \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  tiago_booster
```

### Entrypoint

Al lancio del container, `entrypoint.sh` legge `$ROBOT_STACK` e delega:

| `ROBOT_STACK` | Script eseguito | Cosa fa |
|---|---|---|
| `tiago` | `entrypoint_tiago.sh` | Source ROS2 + workspace TIAGo, imposta CycloneDDS |
| `booster` | `entrypoint_booster.sh` | Imposta LD_LIBRARY_PATH, FastDDS, lancia supervisord o bash |

È possibile passare un comando custom:
```bash
# Tiago con comando custom
docker run -e ROBOT_STACK=tiago -e TIAGO_CMD="ros2 launch tiago_gazebo tiago_gazebo.launch.py" tiago_booster

# Booster con comando custom
docker run -e ROBOT_STACK=booster tiago_booster /app/mio_script.sh
```

### Middleware ROS2

| Robot | RMW | Note |
|---|---|---|
| TIAGo | `rmw_cyclonedds_cpp` | Default PAL Robotics |
| Booster | `rmw_fastrtps_cpp` | Profilo XML: `/app/booster_motion/fastdds_profile.xml` |

---

### TIAGo — Simulazione Gazebo

```bash
# Nel container (ROBOT_STACK=tiago)
ros2 launch tiago_gazebo tiago_gazebo.launch.py
```

Topic principali:

| Topic | Tipo | Descrizione |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | Posizioni giunti |
| `/head_front_camera/rgb/image_raw` | `sensor_msgs/Image` | Telecamera RGB |
| `/head_front_camera/depth/image_raw` | `sensor_msgs/Image` | Profondità |
| `/scan` | `sensor_msgs/LaserScan` | Laser scan |
| `/mobile_base_controller/odom` | `nav_msgs/Odometry` | Odometria |
| `/cmd_vel` | `geometry_msgs/Twist` | Comandi velocità |

---

### Booster T1 — Simulazione Webots

Richiede **3 terminali** nel container (con `docker exec -ti <container> bash`).

**Terminale 1 — Webots**
```bash
start-webots
```
Attendi: `INFO: Waiting for local or remote connection on port 1234`

**Terminale 2 — Runner (controller mck)**
```bash
start-runner
# oppure per variante 7-DOF arms:
start-runner-7dof
```

**Terminale 3 — Controllo locomotion**
```bash
loco
```

Comandi loco:

| Comando | Azione |
|---|---|
| `mp` | Mode: Prepare |
| `mw` | Mode: Walking (stand up) |
| `md` | Mode: Damping |
| `w/s/a/d` | Avanti/indietro/sinistra/destra |
| `q/e` | Ruota sinistra/destra |
| `gu` | Rialzati dal suolo |
| `ld` | Sdraiati |
| `hu/hd/hl/hr/ho` | Testa su/giù/sinistra/destra/centro |

**Sequenza di startup:** `mp` → attendi → `mw`

Topic ROS2 Booster (visibili solo con FastDDS):
```bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 topic list
# /LocoApiTopicReq
# /LocoApiTopicResp
```

---

## Circus — Simulatore MuJoCo

Simulatore fisico multi-robot per RoboCup humanoid. Gestisce la fisica con MuJoCo e il lifecycle dei container Docker.

### Build

```bash
cd circus
pixi build
```

### Run

```bash
pixi run main
```

### Come funziona

Circus gestisce ogni robot come un container Docker separato. Usa la **Docker Engine API** (via Unix socket `/var/run/docker.sock`) per creare, avviare e fermare i container.

Alla creazione del container Circus imposta:

```
ROBOT_NAME=red_Booster-T1_1   # formato: team_TipoRobot_numero
SERVER_IP=172.17.0.1           # IP del bridge Docker
CIRCUS_PORT=5555               # porta TCP per comunicazione
ROBOT_STACK=booster            # seleziona l'entrypoint
```

### Protocollo di comunicazione

Circus e SimBridge comunicano via **TCP + MessagePack** a ~100Hz.

**Container → Circus** (comandi):
```json
{
  "robot_name": "red_Booster-T1_1",
  "joint_torques": [0.0, 0.1, ...]
}
```

**Circus → Container** (stato):
```json
{
  "robot_name": "red_Booster-T1_1",
  "joint_positions": [...],
  "joint_velocities": [...],
  "imu_orientation": [w, x, y, z],
  "imu_angular_velocity": [x, y, z],
  "imu_linear_acceleration": [x, y, z]
}
```

### Robot supportati

| Robot | DOF | Note |
|---|---|---|
| Booster-T1 | 23 | Include giunto vita (Waist) |
| Booster-K1 | 22 | Torso fisso, senza giunto vita |

---

## SimBridge — Ponte ROS2 ↔ Circus

ROS2 node C++ che gira **dentro** il container. Traduce tra i topic ROS2 del framework di controllo e il protocollo TCP di Circus.

### Build

```bash
cd simbridge
pixi install
pixi run build
```

> Non serve eseguirlo manualmente se usi il container: parte automaticamente tramite l'entrypoint.

### Esecuzione standalone (fuori dal container)

```bash
export ROBOT_NAME=red_Booster-T1_1
export SERVER_IP=127.0.0.1
export CIRCUS_PORT=5555
pixi run simbridge
```

### Topic ROS2

| Topic | Tipo | Direzione | Descrizione |
|---|---|---|---|
| `/joint_command` | `sensor_msgs/JointState` | Subscribe | Torques dal framework → SimBridge |
| `/joint_state` | `sensor_msgs/JointState` | Publish | Posizioni/velocità → framework |
| `/imu` | `sensor_msgs/Imu` | Publish | IMU data → framework |

### Flusso interno

1. Il framework pubblica i torques su `/joint_command`
2. SimBridge li serializza in MessagePack e li invia via TCP a Circus
3. Circus avanza la fisica e risponde con lo stato aggiornato
4. SimBridge deserializza e pubblica su `/joint_state` e `/imu`

---

## Flusso completo

```
[Circus GUI]
    │
    │  1. Crea container via Docker API
    │     (ROBOT_STACK=booster, ROBOT_NAME, SERVER_IP, CIRCUS_PORT)
    ▼
[Docker Container]
    │  entrypoint_booster.sh
    │  → imposta LD_LIBRARY_PATH, FastDDS
    │  → avvia SimBridge
    │
    │  2. SimBridge si connette a Circus:CIRCUS_PORT
    │     handshake: invia ROBOT_NAME
    ▼
[Circus RobotManager]  ←──────────────────────────────┐
    │                                                  │
    │  3. Loop ~100Hz                                  │
    │     riceve joint_torques dal container           │
    │     avanza physics MuJoCo                        │
    │     invia joint_state + IMU al container         │
    ▼                                                  │
[SimBridge]                                            │
    │  pubblica /joint_state, /imu                     │
    ▼                                                  │
[Framework controllo]                                  │
    │  calcola controllo                               │
    │  pubblica /joint_command ─────────────────────── ┘
    ▼
    (loop continuo a ~100Hz)
```
