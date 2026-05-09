# Simulator portability

This stack was originally written against the **Udacity Term 3 simulator**. Most of it is generic ROS, but the simulator-facing layer is Udacity-specific. This document captures what's portable, what isn't, and what it would take to drive a different simulator with this code.

## Three layers

### 1. Generic ROS planning/control — fully portable
Works against any simulator that publishes/subscribes the standard topics:

| Node | File | Role |
|---|---|---|
| Path planner | `ros/src/waypoint_updater/waypoint_updater.py` | KDTree-based lookahead waypoint selection |
| Waypoint loader | `ros/src/waypoint_loader/waypoint_loader.py` | Loads CSV waypoints into `/base_waypoints` |
| Pure pursuit | `ros/src/waypoint_follower/` | Twist commands from waypoints (Autoware-derived) |
| TL state arbitration | `ros/src/tl_detector/tl_detector.py` | Combines pose + light positions → `/traffic_waypoint` |
| DBW controller | `ros/src/twist_controller/dbw_node.py` | Twist → throttle/brake/steer |

**Required topics in:** `/current_pose`, `/current_velocity`, `/base_waypoints`, `/image_color`, `/vehicle/traffic_lights`
**Required topics out:** `/vehicle/throttle_cmd`, `/vehicle/brake_cmd`, `/vehicle/steering_cmd`

### 2. `styx` simulator adapter — Udacity-Term-3-specific
This is the only sim-coupled code. Source: `ros/src/styx/server.py` and `ros/src/styx/bridge.py`.

- **Wire protocol:** socket.io server on **port 4567**
- **Inbound events:** `telemetry`, `control`, `obstacle`, `lidar`, `trafficlights`, `image`
- **Outbound events:** `steer`, `throttle`, `brake`, `drawline`
- **Telemetry payload schema** (exact JSON keys the sim sends): `x`, `y`, `z`, `yaw`, `velocity`, `dbw_enable`, `light_pos_x/y/z/dx/dy`, `light_state`, `lidar_x/y/z`, `image` (base64-encoded PNG)
- **Launch hook:** `ros/src/styx/launch/server.launch` calls `unity_simulator_launcher.sh`, which expects the Udacity Linux binary at a known path

### 3. Vehicle bus
`dbw_mkz_msgs` (Dataspeed). Lincoln MKZ-specific message types. The control logic above is generic, but message types are bus-specific.

## Track waypoints
The CSVs in `data/` (e.g., `wp_yaw_const.csv`) are baked for the **Udacity track topology**. Any non-Udacity simulator needs its own waypoint CSV generated from that sim's map.

## Driving a different simulator

| Target | What changes | Effort |
|---|---|---|
| **Gazebo + Dataspeed `dbw_mkz_simulator`** | Skip `styx` entirely; Gazebo publishes the required topics natively. You lose the ground-truth traffic-light feed — either publish your own or use the classifier. | **Lowest** — `roslaunch` change, no code |
| **CARLA** | Use `carla_ros_bridge`, write a remapper that translates CARLA topic names to the names this code expects, and synthesize a `/vehicle/traffic_lights` array | Medium — ~200 lines of adapter Python |
| **SVL / LGSVL** (archived) | Same shape as CARLA — adapt via existing ROS bridge | Medium |
| **AirSim** | Has a ROS wrapper, but vehicle dynamics differ — PID/pure-pursuit needs re-tuning | Medium-high |
| **Any sim speaking the same socket.io protocol** | Would just work | None — none exists publicly |

## TL;DR
The planning/control half is salvageable on any ROS-compatible simulator. The Udacity-specific part is a thin ~200-line socket.io adapter (`styx/`). The lowest-effort path off the Udacity sim is **Gazebo + Dataspeed's MKZ simulator**: drop styx, keep everything else.
