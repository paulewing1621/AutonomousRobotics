# Autonomous Robotics


## System Architecture

```
PC                              TurtleBot3 (SSH)
────────────────────────        ────────────────────────
SLAM Toolbox               ←→   Min Launch
WiFi Map Node              ←→   LiDAR (/scan)
Mission Script                  AutoDock
CA + OccgridPlanner             Task Server
```

---

## Mission Sequence

`mission_dedock_move_dock.py` runs the full autonomous loop:

| Step | Action |
|------|--------|
| 1 | **Undock** from charging station |
| 2 | **Capture TF position** (saved as return target) |
| 3 | **Exploration** for 3 minutes (frontier-based) |
| 4 | **GoToPose** back to saved position |
| 5 | **AutoDock** ×3 retries if needed |

---

## Autonomous Exploration

Frontiers are FREE cells adjacent to UNKNOWN cells. The robot scores and targets them iteratively until the time limit is reached.

**Frontier scoring:**
```
Score = dist × 2.0 + gain × 1.0 − wall_proximity × 0.5
```
- Favors nearby frontiers (`dist`) and those that reveal more surface (`gain`)
- Penalizes frontiers close to walls (`wall_proximity`)

### Components

**`collision_avoidance`**
- Subscribes `/scan`, gates `/commands/velocity`
- `safety_diameter`: 0.5 m
- `ignore_diameter`: 1.0 m
- `max_velocity`: 0.2 m/s
- Always active, independent of planner state

**`occgrid_planner`**
- A* on inflated occupancy grid
- 8-connected neighbourhood
- `robot_radius`: 0.3 m
- Built-in frontier-based exploration explorer
- Enable/disable via `/occgrid_planner/enable_explorer` (SetBool)

**`path_optimizer`**
- Velocity profile smoothing
- `max_acceleration`: 0.3 m/s²
- `max_braking`: 0.1 m/s²

**`path_follower`**
- Carrot following algorithm
- `Kx`: 1.0 · `Ktheta`: 2.0
- `look_ahead`: 1.5 m

---

## WiFi Mapping

Builds an `OccupancyGrid` of WiFi signal strength alongside the SLAM map.

```
/wpa_cli/scan  →  TF Lookup (base_link)  →  /wifi_map_node/wifi_map
(BSSID + dBm)     robot (x, y) position      OccupancyGrid
```

---

## AutoDock

Three-phase return-to-station procedure:

1. Robot positions itself in front of the charging base (GoToPose)
2. Automatic angular alignment
3. Forward approach and dock connection

- **Retry**: up to ×3 — on failure the robot backs up and retries
- `dist_threshold`: 0.3 m · `angle_threshold`: 0.3 rad
- Note: docking consistency depends on angular alignment accuracy from GoToPose

---

## Results

| Metric | Value |
|--------|-------|
| Exploration duration | 3 min (2nd floor) |
| SLAM map resolution | 0.1 m/cell |
| Staircase incidents | 0 |

Complete autonomous loop achieved: **undock → exploration → SLAM + WiFi map → return → dock**

---

## Known Challenges

| Issue | Detail |
|-------|--------|
| Unexplored corridors | Scoring tradeoff between collision avoidance conservatism and exploration reach |
| LiDAR/exploration crashes | Occasional `/scan` topic loss causes random mission stops |
| Exploration service | `/occgrid_planner/enable_explorer` called via `call_async()` — `True` at start of exploration, `False` before GoToPose |
| Parameter tuning | `robot_radius` 0.3 m vs `safety_diameter` 0.5 m — corridors ≥ 1 m navigate reliably |

**Future improvements:** frontier scoring optimization · real-time battery monitoring · smoother velocity profiles

---

## Launch Instructions

### On the TurtleBot (SSH in first)

```bash
ssh turtlebot@turtlebot01
```

Run each of the following in a separate terminal on the robot:

```bash
ros2 launch turtlebot_launch minimal.launch.py
ros2 launch turtlebot_launch rplidar_a1_launch.py
ros2 launch turtlebot_launch autodock.launch.py
ros2 launch turtlebot_launch kinect_min_comp.launch.py
ros2 launch floor_nav launch_server_tb.launch.py
ros2 run topic_tools mux /velocity_smoother/input /teleop/twistCommand /mux/autoCommand --ros-args -r __name:=cmd_mux
ros2 service call /cmd_mux/select topic_tools_interfaces/srv/MuxSelect '{topic: "/mux/autoCommand"}'
ros2 launch wpa_cli wpa_cli.launch.py
```

### On the PC

```bash
ros2 launch turtlebot_launch slam_tb_sync.launch.py use_sim_time:=False
ros2 launch wifi_map_base wifi_map.launch.py
rviz2
ros2 run floor_nav mission_dedock_move_dock.py
```
