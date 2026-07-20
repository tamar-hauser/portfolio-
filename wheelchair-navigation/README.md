# Autonomous Wheelchair Navigation System
### SENSE · MAP · NAVIGATE

`C++` · `Eigen` · `PCL` · `OpenCV` · `Octomap` · `YOLOv8` · `Webots` · `React` · `Node.js` · `SQLite`

Full autonomous navigation system for a wheelchair robot, simulated in **Webots**. Includes a real-time navigation core running at **100Hz** and a web dashboard for monitoring sensor data.

- Fused GPS, IMU, LiDAR, Camera, Radar, and Encoder data using an EKF and Factor Graph (sliding-window optimization) for real-time localization
- Planned global routes with A* on an OSM graph, and replanned locally with D*Lite (incremental, 100ms cycle)
- Built a live 3D Octomap + 2D costmap from LiDAR scans for obstacle avoidance, and tracked dynamic objects with the Hungarian Algorithm + ICP
- Designed a multi-threaded producer-consumer architecture (11 workers) and a React + Node.js monitoring dashboard backed by SQLite

---

## System Overview

```
  IMU(100Hz)  Encoder(100Hz)  GPS(10Hz)  LiDAR(2Hz)  Radar(2Hz)  Camera(2Hz)
      │             │              │           │           │            │
  Ring IMU     Ring Encoder    Ring GPS   Ring LiDAR  Ring Radar  Ring Camera
      └─────────────┴──────────────┴───────────┴───────────┴────────────┘
                                    │
                          SensorConsumer (6 Workers)
                   ┌────────────────┼────────────────┐
              IMU/Encoder        LiDAR            Camera
            direct EKF update  PointCloud          YOLO
                   │                │                │
                   └────────────────┼────────────────┘
                                    │
                       ThreadGrafFector (5 Workers)
                       ┌────────────┴────────────┐
                  SensorFusion + ICP         Factor Graph
                  (TrackedObjects)        ┌────────┼────────┐
                                       IMU/Enc   GPS   LiDAR-ICP
                                      (Δ pose) (abs) (5D rel)
                                          └────────┼────────┘
                                             Optimization
                                          (sliding window)
                                                   │
                                            EKF correction
                                                   │
                              ┌────────────────────┼────────────────────┐
                              │                    │                    │
                           OctoMap 3D           2D Costmap         Wheelchair
                         (3D obstacles)    (LiDAR+Objects+Corridor)  State 8D
                                                   │
                                    ┌──────────────┴──────────────┐
                                  A* Global                   D*Lite Local
                               (OSM graph)               (incremental, 100ms)
                                    └──────────────┬──────────────┘
                                             SecurityWall
                                                   │
                                            WebotsDrive
                                                   │
                                          Motor Commands
```

---

## SENSE – Sensor Fusion & Localization

### Localization Pipeline

Each sensor feeds its own Ring Buffer, consumed by a dedicated worker in `SensorConsumer`. High-frequency sensors (IMU, Encoder) update the EKF directly on every reading. Lower-frequency sensors (LiDAR, Camera, Radar) are packaged into a fused frame and handed to `ThreadGrafFector` for deeper processing.

```
IMU / Encoder ──► direct EKF predict+update  (100Hz)
GPS           ──► absolute factor             (10Hz)
LiDAR         ──► ICP → 5D relative factor    (2Hz)
Camera        ──► YOLO detections             (2Hz)
Radar         ──► object measurements         (2Hz)
```

### EKF – Extended Kalman Filter
- **Predict**: projects the 8D state forward using the motion model; covariance P grows until next measurement
- **Update**: fuses incoming measurement via Kalman Gain; corrects state and reduces uncertainty

### Factor Graph — Sliding Window Optimization
`FactorGraphManager` maintains a sliding window of keyframes (nodes). Each new keyframe is linked to previous ones by factors derived from sensors:

| Factor | Source | Type |
|---|---|---|
| Relative Δ pose | IMU + Encoder | odometry |
| Absolute position | GPS | global anchor |
| Relative 5D pose | LiDAR ICP | scan matching |

The graph is optimized (sparse Cholesky) in the background and feeds a correction term back into the EKF, eliminating drift over time.

### Two Independent State Vectors

| | Wheelchair (8D) | TrackedObject |
|---|---|---|
| State | X·Y·Z·Pitch·Yaw·Vx·Vyaw·Ax | position · velocity · dimensions |
| Inputs | IMU · Encoder · GPS · ICP | LiDAR · Camera · Radar |
| Method | EKF + Factor Graph | SensorFusionM + Hungarian + EKF |
| Rate | 100Hz | 2Hz |

### Multi-Object Tracking — 3-Gate Cost Matrix

Before running the Hungarian Algorithm, each detection passes three gates to build the cost matrix:

| Gate | Criteria |
|---|---|
| Gate 1 | Semantic class + Euclidean distance |
| Gate 2 | Mahalanobis distance + bounding-box size |
| Gate 3 | IoU + velocity consistency + detection confidence |

→ **Hungarian Algorithm** finds the globally optimal assignment of detections to tracked objects
→ 3 outcomes per frame: **update** existing track · **increase** Kalman P (missed) · **create** new object

---

## MAP – Real-Time 3D Mapping

| Layer | Content | Update Trigger |
|---|---|---|
| OctoMap 3D | Probabilistic 3D obstacle map | Every LiDAR scan |
| 2D Grid (2000×2000 @ 0.1m) | High-resolution local occupancy | Every frame |
| LiDAR Layer | Static obstacles | Every LiDAR scan |
| Objects Layer | Dynamic obstacles (TrackedObject pos + radius) | Every frame |
| Global Corridor | Forbidden / high-cost cells outside the sidewalk | Once at map build |
| FinalMap (Costmap) | lidarLayer + objectLayer + corridor → scalar cost per cell | Continuous → D\*Lite |

---

## NAVIGATE – Path Planning & Control

### A\* — Global Planner
- Runs once at startup on a weighted **OSM graph** (nodes = street intersections, edge weight = distance + wheelchair accessibility score)
- Filters / penalizes edges inaccessible to wheelchairs
- Output: sequence of `lat/lon` waypoints passed to D\*Lite

### D\*Lite — Local Planner
- **Incremental replanning**: only recomputes cells whose costmap value changed — not the full grid
- Synchronized to global path: lookahead step advances to the next global waypoint when the current one is reached
- `getNextStep` → linear & angular velocity → `WebotsDrive` → motor commands
- Cycle time: **100ms**

### SecurityWall — Safety Layer
| Condition | Action |
|---|---|
| Dynamic obstacle < 1.5m and approaching | Block immediately |
| Traffic light at intersection: not green or not visible | Block |
| Unmarked crosswalk with dynamic danger | Block |

---

## Threads Architecture

| Thread | Workers | Responsibility |
|---|---|---|
| SensorConsumer | 6 | One worker per sensor type; IMU/Encoder → EKF; LiDAR → PointCloud; Camera → YOLO |
| ThreadGrafFector | 5 | Fused-frame processing (Fusion + ICP); Factor Graph solve; keyframe creation; sliding-window pruning |

---

## Tech Stack

| Category | Technologies |
|---|---|
| Language | C++17 |
| Simulation | Webots |
| Linear Algebra | Eigen3 |
| Point Clouds | PCL |
| Computer Vision | OpenCV · YOLOv8 |
| 3D Mapping | Octomap |
| Database | SQLite |
| Maps | OpenStreetMap (OSM) |
