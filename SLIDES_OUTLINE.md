# Slide Outline — TIAGo + Booster T1 Simulation Stack

---

## Slide 1 — Titolo
**TIAGo & Booster T1: Ambiente di simulazione integrato**
Sottotitolo: Docker · ROS2 · Webots · MuJoCo · Circus · SimBridge

---

## Slide 2 — Obiettivo
- Simulare robot eterogenei (manipolatore + umanoide) nello stesso ambiente
- Isolare ogni robot in un container Docker con il suo software
- Usare lo stesso codice che gira sul robot reale → basso sim-to-real gap
- Abilitare squadre miste in simulazioni RoboCup-style

---

## Slide 3 — Componenti del sistema (overview)

```
┌─────────────┐     Docker API      ┌───────────────────┐
│   Circus    │ ──────────────────► │  Docker Container  │
│ (MuJoCo)   │                     │  TIAGo / Booster   │
│             │ ◄── TCP (5555) ──── │  SimBridge (ROS2)  │
└─────────────┘                     └───────────────────┘
```

4 componenti principali:
1. **Circus** — simulatore fisico (MuJoCo, C++)
2. **Docker container** — ambiente robot isolato
3. **SimBridge** — ponte ROS2 ↔ Circus
4. **entrypoint** — bootstrap dell'ambiente al runtime

---

## Slide 4 — Docker Container
- Immagine unica con **TIAGo** (Gazebo/ROS2) e **Booster T1** (Webots)
- Scelta robot via variabile `ROBOT_STACK=tiago|booster`
- Base: `ros:humble` (Ubuntu 22.04 + ROS2 Humble)
- Middleware separati:
  - TIAGo → **CycloneDDS**
  - Booster → **FastDDS** + profilo XML dedicato

---

## Slide 5 — Entrypoint chain

```
docker run -e ROBOT_STACK=booster image
        │
        └─► /app/entrypoint.sh          ← dispatcher
                  │
                  └─► entrypoint_booster.sh
                        · imposta LD_LIBRARY_PATH
                        · imposta FASTRTPS profile
                        · lancia supervisord o bash
```

- `entrypoint_tiago.sh`: source ROS2 + workspace, unset FastDDS, lancia bash/cmd
- Circus chiama `/app/entrypoint.sh` via Docker API (riga 104 di Container.cpp)

---

## Slide 6 — Circus (simulatore)
- Simulatore MuJoCo multi-robot per RoboCup humanoid
- Scritto in C++ con Qt6 per la GUI
- Build con **pixi** (conda-based, cross-platform)
- Gestisce container Docker via **Docker Engine API** (curl su Unix socket)
- Supporta robot eterogenei nella stessa simulazione

Robot supportati:
- Booster-T1 (23 DOF)
- Booster-K1 (22 DOF)

---

## Slide 7 — Architettura Circus: Container lifecycle

```
Circus                          Docker Engine
  │                                  │
  ├─ POST /containers/create ───────►│  (image, env vars, binds, GPU)
  ├─ POST /containers/{id}/start ───►│
  │                                  │
  │◄── TCP connection (SimBridge) ───┤
  │       handshake: robot_name      │
  │       loop: torques ↔ state      │
  │                                  │
  ├─ POST /containers/{id}/stop ────►│
  └─ DELETE /containers/{id} ───────►│
```

---

## Slide 8 — SimBridge
- ROS2 node C++ che gira **dentro** il container
- Fa da traduttore tra il framework di controllo e Circus
- Build con **pixi** (`pixi run build`)
- Topics ROS2:
  - Sub: `/joint_command` (JointState) — torques dal framework
  - Pub: `/joint_state` (JointState) — posizioni/velocità
  - Pub: `/imu` (Imu) — orientamento, velocità angolare, accelerazione
- Comunicazione con Circus via **TCP + MessagePack** @ 100Hz

---

## Slide 9 — Protocollo di comunicazione

**Container → Circus**
```json
{ "robot_name": "red_Booster-T1_1", "joint_torques": [...] }
```

**Circus → Container**
```json
{
  "robot_name": "red_Booster-T1_1",
  "joint_positions": [...],
  "joint_velocities": [...],
  "imu_orientation": [w,x,y,z],
  "imu_angular_velocity": [x,y,z],
  "imu_linear_acceleration": [x,y,z]
}
```

Serializzazione: **MessagePack** (binario, efficiente)

---

## Slide 10 — Booster T1 in Webots (standalone)
Sequenza di avvio manuale (3 terminali nel container):

| Terminale | Comando | Descrizione |
|-----------|---------|-------------|
| 1 | `start-webots` | Apre Webots, attende connessione porta 1234 |
| 2 | `start-runner` | Connette il controller mck a Webots |
| 3 | `loco` | Client tastiera per controllo locomotion |

Comandi loco: `mp` (prepare) → `mw` (stand up) → WASD per muoversi

---

## Slide 11 — TIAGo in Gazebo (standalone)
```bash
# Nel container con ROBOT_STACK=tiago
ros2 launch tiago_gazebo tiago_gazebo.launch.py
```

Topic principali:
- `/head_front_camera/rgb/image_raw` — telecamera RGB
- `/cmd_vel` — velocità base mobile
- `/joint_states` — stato giunti
- `/mobile_base_controller/odom` — odometria

---

## Slide 12 — Come clonare e avviare

Vedi README.md per istruzioni complete.
Requisiti: Docker + nvidia-container-toolkit + X11

```bash
git clone <repo>
cd tiago_booster_docker
docker build -t tiago_booster .          # ~30-40 min
bash start_tiago.sh                      # oppure start_booster.sh
```

Per Circus e SimBridge:
```bash
cd circus && pixi build && pixi run main
cd simbridge && pixi install && pixi run build
```

---

## Slide 13 — Riepilogo flusso completo

```
[Circus GUI]
     │ lancia container via Docker API
     ▼
[Docker Container] ── entrypoint_booster.sh
     │ SimBridge si connette via TCP
     ▼
[Circus RobotManager] ── loop 100Hz
     │ riceve torques, invia stato
     ▼
[MuJoCo Physics] ── 1000Hz
     │ avanza simulazione
     ▼
[SimBridge pubblica] ── /joint_state, /imu
     │
[Framework controllo] ── pubblica /joint_command
```
