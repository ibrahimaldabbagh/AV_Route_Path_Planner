
# Global Route & Path Planning Module for Autonomous Vehicle (ROS 2/Python)

<p align="center">
  <img src="images/Figure_1.png" width="500">
</p>

---

## 🔍 Overview

This repository implements a **global route & path planner** for an indoor autonomous‑vehicle (AV) model city using **ROS 2 (Python)**. The planner consumes a **topological map** of intersections and roads, computes an **optimal global route** between a start and goal, and outputs an ordered **path** suitable for downstream **trajectory planning** and **control**. In addition to the Cartesian node sequence, it attaches **cardinal direction semantics** (N/E/S/W) that help higher‑level behavior modules reason about turns, approaches, and lane‑like structure.

The module is designed for structured indoor environments (laboratories, model cities, research testbeds) where GPS is unavailable and navigation is based on a **fixed, known graph** of intersections. It integrates tightly with localization (for the current pose), a spot/goal selector (for the target), and a trajectory planner (to generate time‑parameterized motion along the route).

---

## 🎥 Demo

> **Demo** — Example of the AV following a globally planned route inside the indoor model city. The global route is computed from the graph (nodes/edges) and enriched with cardinal directions so the vehicle can recompute routes in consideration of its heading as showen in following demos.

<p align="center">
  <img src="images/route1.gif" width="480">
  <img src="images/route2.gif" width="480">
</p>

---

## ✅ Key Capabilities

- **Global route computation (graph‑based A\*)** over a hand‑authored indoor map  
- **Cardinal‑aware planning** (N/E/S/W) with turn penalties and initial heading consideration  
- **Deterministic path extraction** as an ordered sequence of waypoints (PoseArray)  
- **ROS 2‑native publishers/subscribers** with minimal dependencies and clean interfaces  
- **Separation of concerns**: route computation (global) vs. trajectory planning (local/time‑based)  
- **Tested** with unit and integration tests; launchable via a simple launch file  
- **Visualization** via RQT and static images for route overlays  

---

## 🧭 System Architecture

<p align="center">
  <img src="images/Block Diagram.jpg" width="960">
</p>

**Architecture Summary**

1. **Localization** (`/loc_pose`, `PoseStamped`):  
   Provides current AV position (and heading from a separate `/euler_angles` topic in the full system).
2. **Spot/Goal Selector** (`/selected_spot`, `PoseStamped`):  
   Provides the destination point (e.g., a parking location).
3. **AV_Route_Path_Planner (this module)**:  
   - **Map Loader:** parses `map.txt` and `map_cardinal.txt` from package share.  
   - **Global Planner:** runs a **cardinal‑aware A\*** to compute a node sequence from start to goal.  
   - **Path Publisher:** publishes a **`PoseArray`** on `/route` and a **Bool** `/route_state` indicating availability.
4. **Trajectory Planner** (downstream):  
   Consumes `/route` and produces smooth, time‑parameterized motion.
5. **Behavior Layer / Controller**:  
   Executes the plan and handles actuation.

---

## 🧩 ROS 2 Interfaces

### Subscriptions
| Topic | Type | Purpose |
|---|---|---|
| `/loc_pose` | `geometry_msgs/PoseStamped` | AV’s current localization (x, y, z; used for nearest‑start‑node selection) |
| `/euler_angles` | `geometry_msgs/Vector3` | AV orientation (roll, pitch, yaw) used for initial‑heading penalties |
| `/selected_spot` | `geometry_msgs/PoseStamped` | Target location (used for nearest‑goal‑node selection) |

### Publications
| Topic | Type | Purpose |
|---|---|---|
| `/route` | `geometry_msgs/PoseArray` | **Ordered global path** (waypoints in the `map` frame) |
| `/route_state` | `std_msgs/Bool` | **True** when a route has been generated and published |

**Launch**: `ros2 launch adapt_roucomp roucomp_launch.py`  
**Node Executables**:  
- `route` → basic A\* (Cartesian graph) from `adapt_roucomp.routemodule5:main`  
- `route_cardinal` → enhanced cardinal‑aware A\* from `adapt_roucomp.route_cardinal:main`

---

## 🔢 Map Files & Format

This module relies on two text files, installed into the package share (`share/adapt_roucomp/config/`) and loaded at runtime.

### 1) `map.txt` — Graph Topology (Cartesian)

- **NODES**: each line = `x y id`  
- **EDGES**: each line = `node_id neighbor_id [neighbor_id ...]`  
- **PARKING_SPOTS**: each line = `x y`

**Excerpt (from your `map.txt`):**
```
NODES
0.5 0.7 1
1.5 0.7 2
...
EDGES
1 2 20
2 1 3
...
PARKING_SPOTS
4.75 5.25
4.75 5.75
4.75 6.25
4.75 6.75
```

`routemodule5.py` uses this file to build an adjacency list and perform **Cartesian A\*** on the node graph (closest‑node from vehicle, closest‑node to target).

### 2) `map_cardinal.txt` — Directed Edges (Cardinal)

- **NODES**: `(x, y, id)`  
- **EDGES**: `(start_id, end_id, direction)` where direction ∈ {N, E, S, W}  
- **PARKING_SPOTS**: `(x, y)`

This encoding enables **direction‑aware** planning and turn reasoning.  
Examples:
```
Edges
1, 2, W
1, 20, S
2, 1, E
2, 3, W
...
```

---

## 📍 Map Visualization (Nodes, Edges & Cardinal Directions)

The following figure illustrates the complete **topological graph** used by the route planner.  
Each node represents an intersection/checkpoint in the model city, while each arrow shows a **directed edge** with its associated **cardinal direction**:

- **Red = East (E)**
- **Blue = West (W)**
- **Green = North (N)**
- **Purple = South (S)**

This figure is essential for understanding:
- how the roads are connected,
- where direction transitions occur,
- where turn penalties are applied in the enhanced A\*,
- and how the vehicle can move through the indoor environment.

<p align="center">
  <img src="images/Figure_1.png" width="900">
</p>

---

## 🧭 Cardinal Orientation & Heading

The cardinal‑aware planner attaches **semantic direction labels** to every edge, enabling consistent behavior through intersections (e.g., penalizing U‑turns, encouraging through movements).

<p align="center">
  <img src="images/orientatio_E_1.png" width="920">
  <img src="images/orientatio_w_1.png" width="920">
</p>

In the **enhanced A\***, we apply **direction‑change penalties** (illustrative values):
- **0** — same direction (straight)  
- **5** — right‑angle turn (left/right)  
- **10** — U‑turn

This promotes paths that respect typical road‑behavior rules and avoids unrealistic oscillations at intersections.

---

## 🧠 Algorithmic Flow (A\* with Direction Semantics)

The planner follows a standard best‑first A\* search on the **node graph**, with modifications to incorporate direction:

1. **Preprocessing**  
   Load nodes & edges. If cardinal mode, build a `(from_node, to_node) → {direction}` lookup.
2. **Start/Goal Selection**  
   Snap current pose `(x, y)` to the **nearest node** (start). Snap goal pose `(x, y)` to the **nearest node** (goal).
3. **A\* Loop**  
   Maintain `open_set`, `came_from`, `g_score`, `f_score`. Heuristic: Euclidean distance.
   For each neighbor:
   - `tentative_g = g(current) + dist(current, neighbor)`  
   - If cardinal mode: compute **direction change** from previous edge → add **turn penalty**.  
   - Update `f = tentative_g + heuristic(neighbor, goal)` and push/update neighbor.
4. **Reconstruction**  
   Backtrack `came_from` to produce the final **node sequence (route)**; annotate each step with **N/E/S/W** in cardinal mode.
5. **Publication**  
   Build a `PoseArray` in frame `map` (z=0 for waypoints); publish `/route` and set `/route_state=True` once.

---

## 🛰 RQT Graph (Module Context)

<p align="center">
  <img src="images/rqt_module6.png" width="1100">
</p>

This graph shows the planner’s interaction with localization (`/loc_pose`), orientation (`/euler_angles`), spot selection (`/selected_spot`), downstream trajectory planner (consuming `/route`), and behavior layer (listening to `/route_state`).

---

## 🖼 Route Visualization

<p align="center">
  <img src="images/routecomputer_output.png" width="900">
</p>

The rendered polyline overlays the **final route** across the graph layout. Markers can display node indices and step‑wise cardinal headings for debugging and analysis.

---

## 🧪 Tests (Unit & Integration)

This repository includes both **unit** and **integration** tests to validate parsing, routing, and publication behavior.

### Unit Tests
- **Cartesian planner** (`unit_test.py`): initialization, pathfinding entry points, publisher behavior.  
- **Cardinal planner** (`unit_test_cardinal.py`): `yaw_to_direction`, parsing of node/edge lines, and minimal A\* success cases.

### Integration Tests
- **Cartesian integrator** (`integration_test.py`): route publication on `/route`.  
- **Cardinal integrator** (`integration_test_cardinal.py`): simulates loc/spot input, euler heading, triggers timer, and asserts both `/route` and `/route_state`.

---

## ⚙️ Build & Run

### 1) Build
```bash
cd ~/ros2_ws/src
git clone <REPO_URL> AV_Route_Path_Planner
cd ~/ros2_ws
colcon build --symlink-install --packages-select adapt_roucomp
source install/setup.bash
```

### 2) Run (Cartesian A*)
```bash
ros2 run adapt_roucomp route
```

### 3) Run (Cardinal-aware A*)
```bash
ros2 run adapt_roucomp route_cardinal
```

### 4) Launch
```bash
ros2 launch adapt_roucomp roucomp_launch.py
```

---

## 🛠 Configuration & Parameters (suggested)

While the current implementation uses constants inside the node, we recommend exposing the following as ROS 2 parameters (YAML or CLI):

- `qos_depth` (default: 10) — publisher/subscriber queue depth  
- `route_timer_period` (default: 0.1 s) — route computation timer  
- `map_file` — path to `map.txt` in share  
- `map_cardinal_file` — path to `map_cardinal.txt` in share  
- `turn_penalty_right_angle` (default: 5)  
- `turn_penalty_u_turn` (default: 10)  
- `frame_id` (default: `map`) — for `PoseArray` waypoints  
- `publish_once` (bool) or allow replan on new goals

**Launch example (with params):**
```bash
ros2 launch adapt_roucomp roucomp_launch.py route_timer_period:=0.05 frame_id:=map
```

---

## 🔧 Troubleshooting

- **No route published:** ensure `/loc_pose`, `/euler_angles`, and `/selected_spot` are being published; verify the start/goal are near valid nodes.  
- **“No path found” warnings:** check connectivity in `map.txt`; ensure goal is reachable from start.  
- **Images not showing in README:** confirm the files exist under `images/` with the exact filenames used above.  
- **Cardinal mode not respecting turns:** verify that `map_cardinal.txt` contains correct `(from,to)→direction` entries for all edges.

---

## 📚 Component Description (Formal)

**RouteComputer** calculates the optimal route from the vehicle’s current location to a selected parking spot within a predefined model‑city map. It subscribes to the vehicle’s **position** and **orientation**, and to the target spot coordinates; then publishes a series of **waypoints** that define the route. The algorithm uses **A\*** to compute the shortest path between nodes. In addition, it accounts for **initial orientation** via **turning penalties** to encourage realistic traffic behavior (e.g., avoid unnecessary U‑turns). The component also utilizes a **cardinal‑directional edges map**, enabling A\* to track and penalize direction changes explicitly.

### I/O Summary (Table)

<div align="center">

| In/Out | Topic Name | Message Type | Description |
|---|---|---|---|
| Input | `/loc_pose` | `geometry_msgs/PoseStamped` | Current vehicle position (x, y, z) |
| Input | `/euler_angles` | `geometry_msgs/Vector3` | Current orientation (roll, pitch, yaw) |
| Input | `/selected_spot` | `geometry_msgs/PoseStamped` | Selected parking spot location |
| Output | `/route` | `geometry_msgs/PoseArray` | Optimal route from vehicle to parking spot |
| Output | `/route_state` | `std_msgs/Bool` | Indicates whether a route has been generated |

</div>

---

## 📊 A* Algorithm (Detailed)

This describes the enhanced A\* used in the Route Computer, which incorporates turning penalties and direction mapping to optimize pathfinding.

1. **Start A\*** — initialize data structures.  
2. **Initialize Open Set** — push start node.  
3. **Initialize `g_score` / `f_score`** — costs and estimates.  
4. **Loop while Open Set not empty** — if empty → no path.  
5. **Pick node with lowest `f_score`** — current node.  
6. **If current is goal** — reconstruct and return path.  
7. **For each neighbor of current** — evaluate transitions.  
8. **Compute tentative `g_score`** — base distance cost.  
9. **Add direction penalty** — orientation change cost (0/5/10).  
10. **If improved** — update `came_from`, `g_score`, `f_score`.  
11. **Push neighbor** — add/update in open set.  
12. **Reconstruct path** — backtrack from goal to start.

<p align="center">
  <img src="images/A_algo.png" width="500">
</p>

### A* Algorithm — Sequence (Conceptual)

This sequence shows the interactions in the enhanced A\* pipeline, including penalties and direction mapping.

<p align="center">
  <img src="images/diagram.png" width="700">
</p>

---

## 🧱 Code Components & Interactions (Mind Map)

- **RouteComputerCardinal** — initialization, map loading, A\*, turning penalties.  
- **Metrics Logging** — CSV setup and performance logging.  
- **Map Processing** — nodes/edges/parking; assign parking nodes.  
- **Direction Handling** — yaw→cardinal, change penalties, edge mapping.

<p align="center">
  <img src="images/code_components.png" width="600">
</p>

---

## 🔁 Method Call Flow

From program start to publication and logging:

1. **Program Start** → `__init__`: initialize environment.  
2. **Initialize Components**:  
   - publishers (`/route`, `/route_state`)  
   - subscriptions (`/selected_spot`, `/loc_pose`, `/euler_angles`)  
   - periodic timer → `timer_callback`  
3. **Load Map**:  
   - `initialize_metrics_logging`  
   - `process_node_line` / `process_edge_line` / `process_parking_spot_line`  
4. **Timer Callback**:  
   - `a_star` (enhanced)  
   - assign parking nodes  
   - `log_metrics`  
5. **Publish**:  
   - `publish_route` and `publish_route_state`

<p align="center">
  <img src="images/Method_Call_Flow.png" width="800">
</p>

---

## 🗺 Map.txt — Full Explanation

**Nodes** (`x y id`), **Edges** (`id neighbors...`), and **Parking Spots** (`x y`) define the Cartesian graph:

```
NODES
0.5 0.7 1
1.5 0.7 2
2.3 0.4 3
3.5 1.0 4
3.50 1.5 5
3.50 2.7 6

EDGES
1 2 20
2 1 3
3 2 4
4 3 5 21
5 4 6
6 5 7 30 29

PARKING_SPOTS
4.75 5.25
4.75 5.75
4.75 6.25
4.75 6.75
```

---

## 🧭 Map_cardinal.txt — Full Explanation

**Nodes** `(x, y, id)`, **Edges** `(start, end, dir)`, **Parking Spots** `(x, y)`:

```
NODES
0.5, 0.7, 1
1.5, 0.7, 2
2.3, 0.4, 3
3.5, 1.0, 4
3.50, 1.5, 5
3.50, 2.7, 6

Edges
1, 2, W
1, 20, S
2, 1, E
2, 3, W
3, 2, E
3, 4, W

Parking Spots
4.75, 5.25
4.75, 5.75
4.75, 6.25
4.75, 6.75
```

---

## 🛰 RQT — Module Context (Narrative)

The RQT graph shows how the system collaborates to guide the ego‑vehicle to a parking spot. The **localization** node provides the vehicle’s current position and orientation; the **spot_finder** publishes the selected parking spot; the **route_computer_cardinal** calculates the global route to that spot; the **trajectory_planner** consumes the route to generate motion; and the behavior/decision layer monitors `/route_state` to know when the route is ready.

<p align="center">
  <img src="images/rqt_module6.png" width="1800">
</p>

---

## ⚙️ Functionality (Concise)

1. Subscribe to `/loc_pose`, `/euler_angles`, and `/selected_spot`.  
2. Compute a route with enhanced A\* (orientation at start + turn penalties) using `map_cardinal.txt`.  
3. Publish waypoints (`PoseArray`) to `/route` and readiness on `/route_state`.

<p align="center">
  <img src="images/relative_path.png" width="810">
</p>

This diagram represents nodes from `map.txt` and relative path connections.

---

## 📈 Demo & Results (Orientation Cases)

Tests were run from the origin with different initial headings:

**Case 1 — Initial heading: South**  
<p align="center">
  <img src="images/south.png" width="400">
</p>

**Case 2 — Initial heading: West**  
<p align="center">
  <img src="images/west.png" width="400">
</p>

Even with **identical starting positions**, the planner adjusts waypoints based on **initial orientation**, producing routes that exhibit realistic turning behavior.

---

## 📂 Repository Layout

```
AV_Route_Path_Planner/
├── route_cardinal.py              # Cardinal-aware planner (global route + directions)
├── routemodule5.py                # Cartesian A* planner and ROS 2 node
├── roucomp_launch.py              # LaunchDescription: runs RouteComputer
├── map.txt                        # Node/edge/parking map (Cartesian)
├── map_cardinal.txt               # Directional map (cardinal semantics)
├── integration_test.py            # End-to-end ROS 2 integration test
├── integration_test_cardinal.py   # Integration test for cardinal planner
├── unit_test.py                   # Unit tests (Cartesian planner)
├── unit_test_cardinal.py          # Unit tests (cardinal planner: yaw→dir, parsing, edges)
├── setup.py                       # Packaging (installs config files into share/)
└── README.md                      # This document
```

Notes:
- `setup.py` installs `config/map.txt` and `config/map_cardinal.txt` into `share/adapt_roucomp/config/` and exposes console scripts `route` and `route_cardinal`.
- The launch file spawns the node `routemodule5` from package `adapt_roucomp`.

---

## 🧪 Tests (Unit & Integration)

This repository includes both **unit** and **integration** tests to validate parsing, routing, and publication behavior.

### Unit Tests
- **Cartesian planner** (`unit_test.py`): initialization, pathfinding entry points, publisher behavior.  
- **Cardinal planner** (`unit_test_cardinal.py`): `yaw_to_direction`, parsing of node/edge lines, and minimal A\* success cases.

### Integration Tests
- **Cartesian integrator** (`integration_test.py`): route publication on `/route`.  
- **Cardinal integrator** (`integration_test_cardinal.py`): simulates loc/spot input, euler heading, triggers timer, and asserts both `/route` and `/route_state`.

---

## 🛠 Installation Instructions

**Step 1 — Create workspace**
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
```

**Step 2 — Clone repository**
```bash
git clone <REPO_URL> AV_Route_Path_Planner
```

**Step 3 — Build package**
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select adapt_roucomp
source install/setup.bash
```

**Step 4 — Run executables**
```bash
ros2 run adapt_roucomp route
ros2 run adapt_roucomp route_cardinal
```

**Step 5 — Launch file**
```bash
ros2 launch adapt_roucomp roucomp_launch.py
```

---

## 👤 Maintainer

**Ibrahim Aldabbagh** — Robotics & Perception Engineer
> ⚠️ **Portfolio — Proprietary Notice**
> 
> This repository is part of my personal portfolio — **viewing only**.  
> Do **not** copy, fork, modify, reuse, or redistribute any files without written permission.  
> © 2025 Ibrahim Aldabbagh — **All rights reserved.**
