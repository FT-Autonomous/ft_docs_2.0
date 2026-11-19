# GraphSLAM

*ft_state_estimation · iSAM2 implementation*

---

## What is GraphSLAM?

Graph-based SLAM uses a graph to represent the environment and the vehicle's pose estimates. It answers two questions simultaneously: **where am I?** and **what does the world look like?**

The algorithm works by:

- Initialising a map with some prior knowledge
- Constructing a factor graph as the car drives
- Adding constraints each time a cone is re-observed
- Optimising to find the set of poses and cone positions that best satisfies all constraints simultaneously

![GraphSLAM front-end and back-end](../../media/state_estimation/graphslam_frontend_backend.png)

> **Primary goal:** Build a high-fidelity map in Lap 1. Once finalised, the map serves as the ground truth for Lap 2 autonomous racing.

### Intuition

- Every node in the graph corresponds to a pose of the robot or a landmark position
- Every edge between two nodes represents a spatial constraint between them
- GraphSLAM builds the graph and finds the node configuration that minimises the error across all constraints

![Minimise the differences](../../media/state_estimation/graphslam_minimise.png)

The best configuration ensures that the real and predicted observations are as similar as possible.

---

## Why We Need SLAM

The car cannot race optimally without a consistent, high-fidelity map of the track.

| Lap 1 — Discovery | Lap 2 — Execution |
|---|---|
| Build the cone map from scratch. Estimate vehicle pose relative to unknown landmarks. | Path planning uses the finalised map for racing line optimisation. SLAM is turned off once the course is known. |

Every cone position error in the Lap 1 map degrades the Lap 2 racing line.

![Without SLAM vs With SLAM](../../media/state_estimation/graphslam_slam_analogy.png)

---

## Why We Switched from FastSLAM

Three specific problems in last year's system made switching necessary.

### Unmaintainable codebase

The legacy FastSLAM implementation was highly complex and non-idiomatic. Debugging it was slow and extending it for FSUK-specific needs was nearly impossible.

### Performance at competition

FastSLAM's per-frame cost scales with **#particles × #associations**. Under ambiguous cone conditions the filter needs more particles to stay stable, causing worst-case CPU spikes that produced control-loop lag at last year's competition. The "Fast" in FastSLAM was not living up to its name.

### Non-repeatable maps

Because particle filtering is stochastic, running the same rosbag twice could produce different maps. That makes it impossible to reliably validate the racing line SLAM is feeding to path planning.

---

## Filtering vs. Smoothing

**FastSLAM uses recursive filtering** — each new measurement updates the current estimate and discards the past. Think of it as a broken telephone: each step only knows the last message. Errors that creep in early are very hard to undo.

**GraphSLAM uses global smoothing** — all poses and landmarks are kept in a single factor graph and optimised together. Think of it as a jigsaw puzzle: you keep all the pieces on the table and later observations can pull earlier estimates into better agreement.

![Filtering vs Smoothing](../../media/state_estimation/diagram_filtering_vs_smoothing.png)

The critical property for FSUK is the **backward correction**. When the car re-observes a cone it has seen before — which happens constantly as it goes around a bend — that re-observation propagates corrections back through the entire trajectory, not just the current frame. This is what gives a consistent map at the end of Lap 1.

---

## Current Implementation

`GraphSLAMEngine` (`slam_engine.py`) is the core. It has no ROS dependencies — pure Python, independently testable and benchmarkable. The ROS node (`slam_node.py`) is a thin wrapper that subscribes to topics, calls the engine, and publishes results.

The code is split into three stacks:

- **Live race stack** — `slam_node`. The only part needed to race.
- **Live debug** — `slam_evaluate` and `slam_plotter`. Run alongside the race stack for plots, metrics and diagnostics (scoring only works in sim).
- **Offline tuning** — `slam_replay`. Replays recorded or simulated bags through the same engine and scoring.

See [GraphSLAM Flows](graphslam_flows.md) for a flow-based walkthrough of each in the [GraphSLAM IcePanel](https://s.icepanel.io/gubM8m5BKYbbkm/Z5CD/landscape/diagrams/viewer?diagram=GdQAJT7l7c).

### Topics

`slam_node` subscribes to:

| Topic | What it is |
|---|---|
| `/cones` | Cone positions and colours from Perception |
| `/gnss_pose` | Car position and heading from the Xsens GPS/INS |
| `/gnss` | Raw GPS status, only used to wait for a proper fix before starting |

In sim (`use_sim:=true`) it swaps the GPS for `/odometry_integration/car_state` and `/ground_truth/odom`.

`slam_node` publishes:

| Topic | What it is |
|---|---|
| `/slam/map` | The cone map: x, y and colour of every confirmed cone |
| `/slam/pose` | Current car pose: x, y and heading |
| `/slam/path` | Where the car has been (for RViz) |
| `/slam/state` | Lap count, and whether the map is done (`MAPPING` → `LOCALISING`) |

No speed or velocity is published, just position and heading.

> **Note:** Path planning and control still subscribe to `/map` and `/pose` (the old FastSLAM names), so the `/slam/` topics are not plugged in yet. From `slam_node.py`: "TODO: Let Path Planning know of new topics."

### Factor Graph

Each frame, three types of element are added to the graph:

| Element | What it represents |
|---|---|
| `X(i)` — pose node | Vehicle pose (x, y, θ) at timestep i. One per cone message. |
| `L(j)` — landmark node | Global position (x, y) of physical cone j. |
| `PriorFactor` on `X(0)` | Anchors the origin. Very tight sigma (0.001 m) — critical for preventing map drift. |
| `BetweenFactor` `X(i-1) → X(i)` | Relative odometry constraint. Encodes how much we trust the pose source. |
| `BearingRangeFactor2D` `X(i) → L(j)` | One observation: bearing and range from pose i to cone j. |

![Factor Graph](../../media/state_estimation/diagram_factor_graph.png)

Squares are factors (constraints), circles are variable nodes. Every time the car re-observes a cone, a new `BearingRangeFactor` is added linking the current pose to that same landmark node. These re-observations are the jigsaw connections that pull everything into agreement.

### iSAM2 Solver

Rather than re-solving the full graph from scratch each frame, iSAM2 maintains a Bayes tree and re-linearises only the subtree affected by the new factors. In practice the per-frame cost stays roughly constant as the map grows, because most of the tree is untouched. This is the key property that makes it feasible on a Jetson.

Two parameters control the accuracy/speed trade-off:

- `relinearize_threshold` (default 0.1) — how much a variable must change before its neighbours are re-linearised. Lower = more accurate, slower.
- `relinearize_skip` (default 1) — how many updates can pass before a forced re-linearisation.

Both are marked TODO and need calibrating against real hardware data.

### Nearest-Neighbour Data Association

Each observation is projected into global coordinates using the current pose estimate, then compared against all known landmarks by Euclidean distance. If the closest landmark is within the gate (`association_dist = 1.7 m`), the observation is matched to it. Otherwise a new landmark is created.

![Data Association](../../media/state_estimation/diagram_assoc.png)

The colour constraint is applied before the distance check:

- Blue (1) will not match yellow (2)
- Unknown-colour observations (0) match anything
- If a landmark was first seen without a colour and a later observation has a colour, the landmark's colour is updated

> **Note:** Colour encoding: 0 = unknown, 1 = blue, 2 = yellow/gold, 3 = orange. Confirm with path planning whether colour 2 should be labelled "yellow" or "gold" before integration.

We use Euclidean rather than Mahalanobis distance for the same reason as the old FastSLAM refactor — the likelihood threshold was hard to tune reliably. Mahalanobis is worth revisiting if spurious duplicates become a problem after noise tuning.

### Per-Frame Processing

Triggered by each incoming `/cones` message:

![Per Frame Flow](../../media/state_estimation/diagram_per_frame.png)

1. **Odometry pose arrives** → `add_keyframe(odom_pose)`
2. **Convert cones body → (bearing, range)** → `math.atan2(y,x)`, `hypot(x,y)`
3. **Project observation to global coords** → `_project(pose_idx, b, r)`
4. **Nearest-neighbour data association** → `_associate()` — colour gated
5. **Add BearingRangeFactor2D to graph** → `add_observation()`
6. **iSAM2 incremental update** → `optimize()` — Bayes tree update
7. **Publish `/slam/map` · `/slam/pose` · `/slam/path` · `/slam/state`** → `_publish()`

A cone only goes out on `/slam/map` once it has been seen twice (`min_observations`), or once if it has a colour (`confirm_colored_immediately`).

### Startup

The node waits for a GPS fix (or sim odometry) before any cones count. The first pose gets pinned as the origin. On the real car the first valid fix "anchors the local ENU frame" (`slam_node.py`); lat/lon is converted to local east/north in `conversions.py`.

### Laps and Map Confidence

After 15 m of driving (`loop_closure_min_travel`), getting back within 2 m (`loop_closure_dist`) and ~20° (`loop_closure_heading_tol`) of the start counts a lap.

The map is done when a lap is closed, 90% of cones are confirmed (`confirmed_fraction_threshold`) and almost no new cones are showing up (`new_landmark_rate_threshold`, over the last `confidence_window` keyframes). Then `/slam/state` flips to `LOCALISING` and stays there.

### Parameters

All tunable values live in `DEFAULT_PARAMS` at the top of `slam_engine.py`. Override by passing a dict to the constructor. All noise values are conservative starting points that need calibrating against real data.

| Parameter | Default / Notes |
|---|---|
| `prior_sigmas` | `[0.001, 0.001, 0.001]` m/m/rad — keep tight or the map drifts |
| `odom_sigmas` | `[0.05, 0.05, 0.035]` m/m/rad — increase if the ZED pose is noisy |
| `obs_sigmas` | `[0.1, 0.5]` rad/m — decrease if the sensor is more accurate than assumed |
| `association_dist` | `1.7` m — decrease on densely packed tracks to avoid cross-matching |
| `relinearize_threshold` | `0.1` — TODO: tune on Jetson |
| `relinearize_skip` | `1` — TODO: tune on Jetson |
| `min_observations` | `2` — sightings before a cone is published |
| `confirm_colored_immediately` | `True` — publish coloured cones on first sight |
| `loop_closure_dist` | `2.0` m — distance gate to start pose |
| `loop_closure_heading_tol` | `0.35` rad (~20°) — heading gate to start pose |
| `loop_closure_min_travel` | `15.0` m — travel required to arm the closure check |
| `confidence_window` | `30` — trailing keyframes for the growth-rate calc |
| `confirmed_fraction_threshold` | `0.9` — fraction of tracked landmarks that must be confirmed |
| `new_landmark_rate_threshold` | `0.05` — max fraction of in-window observations that are new |

On the real car `slam_node.py` overrides the odometry noise with `REAL_CAR_ENGINE_PARAMS = {"odom_sigmas": [0.3, 0.3, 0.05]}`.

---

## Testing

### Unit Tests

Test suite is in `tests/test_engine.py`. Covers init, keyframe insertion, optimisation, data association, colour constraint, and a straight-drive integration test. Runs without ROS:

```bash
uv run pytest ft_state_estimation/slam/tests/test_engine.py -v
```

![Simulation results](../../media/state_estimation/graphslam_sim_results.png)

### Integration Testing in the Simulator

Run at slow speed — path planning and control are not perfect at higher speeds and the car can go off-track.

```bash
colcon build --packages-select ft_state_estimation
source install/setup.bash
ros2 launch ft_state_estimation slam.launch.py

# separate terminal
ros2 bag play <path_to_bag> --clock
```

Launches `slam_node`, `slam_plotter` and `slam_evaluate` together. The evaluator saves `eval_gt.png` and `eval_gt_vs_slam.png` when the bag stops for 3 s. The plotter and evaluator are not needed to race; turn them off with `enable_plotter:=false` and `enable_evaluate:=false`.

`slam_plotter` subscribes to `/slam/map`, `/slam/pose` and raw odometry (`/odometry_integration/car_state` in sim, `/zed/zed_node/pose` on the car). The black star is where SLAM thinks the car is, the red star is raw odometry; the gap is how much SLAM is correcting. It saves `slam_live.png` on Ctrl+C.

![Live plotter and ground truth comparison](../../media/state_estimation/graphslam_live_eval.png)

> **Note:** Ground truth validity has not been fully confirmed. Verify `/ground_truth/cones` is publishing sensible data before drawing conclusions from the precision/recall numbers.

### Offline Replay

`slam_replay` replays rosbag recordings from real runs through the engine. It is not a ROS node, so it doesn't subscribe to anything — it reads the bag straight through in order. No messages are dropped, so the same bag gives the exact same map every time. It runs the same GraphSLAM code as `slam_node` and the same scoring as `slam_evaluate`.

```bash
python3 -m ft_state_estimation.slam.slam_replay bags/full_track_drive1 --plot
```

The loop: record a run → replay it → read the scores → change params → replay again. Params live in `DEFAULT_PARAMS` (`slam_engine.py`) and `REAL_CAR_ENGINE_PARAMS` (`slam_node.py`).

### Evaluation Metrics

The evaluator subscribes to `/slam/map`, `/ground_truth/cones` and `/odometry_integration/car_state`. Each SLAM cone can match one GT cone within 1.0 m (`MATCH_THRESHOLD`), and the colour has to match. It reports:

- **Precision** — fraction of SLAM landmarks within 1.0 m of a GT cone
- **Recall** — fraction of GT cones within 1.0 m of a SLAM landmark
- **Mean / median error** — average positional error across all GT cones
- **False positives** — SLAM landmarks with no nearby GT cone
- **Missed cones** — GT cones with no nearby SLAM landmark

---

## Pending Tasks

- **Noise tuning** — all sigma values are placeholders; calibrate against real rosbag data
- **Keyframe gating** — add a minimum-displacement gate before `add_keyframe()` to prevent graph bloat at high observation rates
- **Intra-batch deduplication** — two cones in the same message can create duplicate landmarks; add a second-pass check over pending new landmarks per batch
- **Odometry topic** — confirm whether `/odometry_integration/car_state` exists in the current sim; update to `/imu/data` if not (raised by abkhan04 in PR #2)
- **Huber robust factors** — down-weight outlier observations so one bad perception frame does not dominate the solution
- **Smart cone filtering** — gating + temporal consistency checks to reject ghost cone detections
- **RViz path testing** — `/slam/path` publishes up to 10,000 poses per frame; test for memory/rendering issues on long bags
- **Path planning topics** — path planning and control still subscribe to `/map` and `/pose`; move them to `/slam/map` and `/slam/pose`
- **RTK integration** — `slam_node.py` now uses `/gnss_pose`; `slam_plotter.py` still has "TODO: Change once RTK setup" and uses `/zed/zed_node/pose`
- **Performance profiling** — profile `optimize()` on the Jetson; not yet benchmarked on the actual stack

---

## Extra

### Definitions

- **Yaw** — the angle of the car's nose relative to its starting position
- **Quaternion** — 4D mathematical representation of orientation: (x, y, z, w)
- **Odometry** — a rough sense of how far you've moved and in what direction; drifts over time
- **Bearing** — the angle to an object relative to the direction the robot is currently facing

### Constraints recap

- **Prior Factors** — anchors the start position (low uncertainty)
- **Between Factors** — represents odometry motion between poses
- **BearingRange Factors** — connects poses to landmarks based on sensor data
- **Data Association** — uses Euclidean distance and colour matching to determine if a cone is new or a re-observation

### Mahalanobis vs Euclidean Association

Mahalanobis accounts for the uncertainty in both the projected observation and the stored landmark, giving a probabilistically correct gate. Euclidean ignores uncertainty and just uses position. In practice Euclidean works fine when the pose estimate is accurate. If spurious duplicates become a problem after noise tuning, Mahalanobis is worth revisiting.

### Log-Likelihood Note

If the likelihood-based association approach is ever investigated, use log-likelihood rather than calling `np.log` on the likelihood value. The log-likelihood turns the product of likelihoods into a sum, which is numerically stable and avoids underflow when many observations accumulate.

### Alternative Backend

GraphSLAM is fundamentally a nonlinear least-squares problem. If GTSAM ever becomes a dependency headache, the backend could be re-implemented using SciPy or JAX. GTSAM is the right call for now — it provides well-tested iSAM2 and a clean factor graph API.

---

## slam_engine.py — Function Reference

### `__init__(self, params)`

Initialises the engine and the GTSAM iSAM2 solver. Sets up `_graph` (holds mathematical constraints) and `_initial` (holds first-guess positions). Converts sigma values into GTSAM noise models.

### `add_keyframe(self, odom_pose)`

Records a new vehicle pose. For the very first pose (`i == 0`) it adds a **Prior Factor** to anchor the car to the starting position. For every subsequent pose it calculates the delta from the previous pose and adds a **Between Factor**.

### `add_observation(self, pose_idx, bearing, range_, color)`

Connects the current pose to a cone observation. Calls `_associate()` to check if the cone has been seen before. If it has, adds a factor to the existing landmark. If not, calls `_project()` to initialise a new landmark node in the graph.

### `optimize(self)`

Pushes new factors and initial values into iSAM2. The solver reconciles all constraints and updates the full estimate. Clears `_graph` and `_initial` afterwards — iSAM2 absorbs them internally so they must not be sent twice.

### `get_current_pose(self)`

Returns the latest optimised `(x, y, θ)`. Falls back to raw odometry if `optimize()` has not been called yet.

### `get_all_landmarks(self)`

Returns a list of every cone found: `(idx, x, y, color)`. This is what path planning uses to determine where to drive.

### `get_pose_history(self)`

Returns the full list of optimised poses since start — useful for drawing the car's trajectory in RViz.

### `get_confidence(self)`

"Loop-closure + map-growth confidence snapshot for diagnostics." Returns `loop_closed`, `laps`, `confirmed_fraction`, `new_landmark_rate` and `confident` (map considered "done"), plus closure-gate diagnostics.

### `_get_pose(self, idx)` *(private)*

Returns the best available estimate for a specific pose — optimised result first, raw odometry as fallback.

### `_project(self, pose_idx, bearing, range_)` *(private)*

Converts a camera-frame observation into global map coordinates. Example: if the car is at `(10, 10)` facing north and sees a cone 2 m ahead, this returns `(10, 12)`.

### `_associate(self, pose_idx, bearing, range_, color)` *(private)*

Decides if an observation matches an existing landmark or is a new cone. Projects the observation into the map and searches for any existing landmark within `association_dist`. Also enforces the colour constraint.

---

## References

1. [GTSAM library (borglab/gtsam)](https://github.com/borglab/gtsam)
2. Kaess et al., *iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree*, IJRR 2012 — [link](https://journals.sagepub.com/doi/10.1177/0278364911430419)
3. [Previous FastSLAM implementation — FT-FastSLAM-2023](https://github.com/FT-Autonomous/FT-FastSLAM-2023)
4. [SLAM performance analysis referenced in decision slides](https://uclalemur.com/blog/slam-performance-analysis)
5. [Cyrill Stachniss SLAM lectures — YouTube](https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_)
6. [Probabilistic Robotics — Thrun, Burgard, Fox](https://docs.ufpr.br/~danielsantos/ProbabilisticRobotics.pdf)
