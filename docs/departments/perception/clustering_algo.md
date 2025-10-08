# LiDAR-Only Perception Pipeline

The **LiDAR-only perception pipeline** is responsible for detecting and localizing cones around the vehicle using only 3D point clouds from a LiDAR sensor (e.g., Velodyne or Livox).  
It filters the point cloud, removes the ground plane, clusters cone-like objects, estimates their colors, and publishes results for downstream modules such as planning or mapping.

For information, the track has cones on its left and right.

- Cones on the left are blue
- Cones on the right are yellow
- Just remember the acroynm **BOY**
  - **B**lue on left, **Y**ellow on right. :)

!!!note
    Each heading may have two ways of explainations: One is using just text, and the other using code snippets. So there might be redundancies.

---

## Overview

This system receives a continuous stream of 3D point clouds from the vehicle’s LiDAR sensor and processes them through a series of stages:

1. **Input & Filtering** – Reads the incoming LiDAR data and keeps only the region of interest around the car.  
2. **Ground Plane Removal** – Uses a mathematical plane model to remove road surface points.  
3. **Clustering** – Groups nearby points into objects that likely represent cones.  
4. **Color Classification** – Estimates cone color using intensity or geometric patterns.  
5. **Publishing** – Sends the detected cone positions to other modules in standardized ROS 2 message formats.

The result is a reliable and self-contained perception layer that functions without any camera input.

```text
LiDAR PointCloud2
   ↓
Geometric & RANSAC Filtering
   ↓
Ground Plane Removal
   ↓
Clustering (Cone Candidates)
   ↓
Color Classification
   ↓
ConeArrayWithCovariance (/cones/lidar)
```

---

## Node Summary

This functionality is implemented in the **`Misperception_Node`**, written in Python using ROS 2’s `rclpy` library.  
The node subscribes to LiDAR topics, processes each point cloud, and publishes cone detections.

Its main processing function is the **LiDAR callback**, which automatically runs every time a new point cloud message is received.

| Property | Description |
|-----------|-------------|
| **Node name** | `Misperception_Node` |
| **File** | `misperception_node.py` |
| **Purpose** | Detect cones purely from LiDAR data |
| **Framework** | ROS 2 (`rclpy`) |
| **Main function** | `lidar_cb()` |

---

## Subscriptions (Inputs)

The node listens to two key topics:

- **LiDAR Point Cloud** – A continuous feed of 3D points from the sensor, each with position and intensity values.  
- **Ground Plane Model** – A set of coefficients representing the flat ground surface, used to distinguish between road points and above-ground objects.

The LiDAR topic is the raw sensor input, while the plane model provides context for removing the ground.

| Topic | Type | Description |
|-------|------|-------------|
| `/velodyne_points` | `sensor_msgs/PointCloud2` | Raw LiDAR point cloud (x, y, z, intensity). |
| `/cluster/plane_model` | `std_msgs/Float64MultiArray` | Ground plane coefficients `[a, b, c, d]` used for removing ground points. |

!!! warning
    The node will **not run** if the plane model has not been published yet.

---

## Publications (Outputs)

After processing, the node produces several outputs:

- A **Cone Array**, which lists all detected cones with their positions.  
- **Filtered Point Clouds**, which show the LiDAR data after various stages such as ground removal or clustering.  
- **Debugging Point Clouds**, which help visualize intermediate results in tools like RViz.

Each of these outputs helps validate and visualize how well the perception system is performing.

| Topic | Type | Description |
|-------|------|-------------|
| `/cones/lidar` | `eufs_msgs/ConeArrayWithCovariance` | List of detected cone centroids (currently all in `unknown_color_cones`). |
| `/cluster/filtered_point_cloud` | `sensor_msgs/PointCloud2` | LiDAR points after removing the ground plane. |
| `/cluster/cluster_cloud` | `sensor_msgs/PointCloud2` | All clustered points (candidate cones). |
| `/cluster/cluster_point_cloud` | `sensor_msgs/PointCloud2` | Republished raw cloud for debugging. |
| `/cluster/ransac_point_cloud` | `sensor_msgs/PointCloud2` | Points used for ground model fitting validation. |

---

## Parameters

The node has a number of configurable parameters that define its working region and filtering limits.

They include horizontal boundaries (left and right), distance thresholds (how far forward or backward to process), and elevation limits (which heights above the ground to consider).  

These parameters let the system focus only on the relevant space around the car, improving both efficiency and accuracy.

There are also internal constants that define cluster size limits, ground plane thresholds, and cone height expectations.

These parameters control geometric filtering and limits. They can be overridden in launch files or YAML configs.

| Parameter | Default | Unit | Description |
|------------|----------|------|-------------|
| `LEFT` | `5.0` | m | Left lateral boundary. |
| `RIGHT` | `-5.0` | m | Right lateral boundary. |
| `MIN_DISTANCE` | `-5.0` | m | Minimum forward distance. |
| `MAX_DISTANCE` | `25.0` | m | Maximum forward distance. |
| `MIN_ELEVATION` | `-3.0` | m | Minimum z-filter threshold. |
| `MAX_ELEVATION` | `2.0` | m | Maximum z-filter threshold. |
| `CAMERA_HEIGHT` | `0.3` | m | Declared (unused, placeholder for future camera fusion). |

### Internal Constants

| Variable | Value | Meaning |
|-----------|--------|---------|
| `CLUSTER_DISTANCE_WIDTH` | 0.5 m | Max XY distance between points in a cluster. |
| `CLUSTER_DISTANCE_HEIGHT` | 4 m | Max Z distance allowed in a cluster. |
| `CLUSTER_HEIGHT_MINLIMIT` | 0.1 m | Minimum vertical size of a valid cone. |
| `CLUSTER_HEIGHT_MAXLIMIT` | 0.5 m | Maximum vertical size of a valid cone. |
| `GROUND_THRESHOLD` | 0.05 m | Distance band for plane removal. |

---

## Step-by-Step Processing

### 1. Reading LiDAR Data

Each incoming LiDAR message is converted into a usable data structure containing the position (x, y, z) and intensity of each point.  

This conversion allows the system to perform numerical operations like distance calculations and clustering efficiently.

Each incoming message is converted from `PointCloud2` → NumPy array `(N×4)` containing:
```
[x, y, z, intensity]
```

The `read_lidar_points()` helper handles parsing the binary ROS message into float arrays.

---

### 2. Region-of-Interest Filtering

The pipeline first limits the incoming data to a **region of interest** — typically a rectangular area directly around the car.  
This step removes far-away points and irrelevant background elements such as the sky or the pit wall.

Only points that fall within specific lateral, forward, and vertical boundaries are kept.  
The result is a smaller, more focused dataset containing the track area where cones are expected.

Points outside a configurable box around the vehicle are removed.

```python
if RIGHT ≤ y ≤ LEFT and MIN_DISTANCE ≤ x ≤ MAX_DISTANCE and z ≤ MAX_ELEVATION:
    keep point
```

This removes far, irrelevant, or sky points before further processing.

**Output:** A trimmed cloud focused on the track in front of the car.

---

### 3. Ground Plane Removal

Once the region of interest is isolated, the system removes the ground points.  
It uses a plane model (defined by four coefficients) to calculate how far each LiDAR point is from the road surface.  

Points that are too close to the plane are considered part of the ground and are discarded.  
Points above the plane remain, as they are likely to represent cones or other obstacles.

This stage ensures that only meaningful objects — those standing on the track surface — are retained for clustering.

The node waits for a `/cluster/plane_model` message describing the road surface:
```
ax + by + cz + d = 0
```

Each point’s **signed distance** from the plane is computed:
```
distance = (a*x + b*y + c*z + d) / sqrt(a² + b² + c²)
```

Points within ±5 cm of the plane are treated as ground and discarded.  
Remaining points above the plane (e.g., cones) are kept.

!!! note
    The sign of the plane normal is checked to ensure correct "above ground" direction using a test point below the car (`[0, 0, -10]`).

---

### 4. Clustering Cone Candidates

After the ground has been removed, the remaining points are grouped into clusters.  
Each cluster represents a collection of points that are close together in space and are therefore likely part of the same physical object.

The clustering algorithm repeatedly selects a point, finds all neighbors within a small radius, and groups them.  
Each group’s centroid and height are then calculated.  
Clusters that are too short or too tall to be cones are filtered out.

The result is a set of cone-like clusters, each with an estimated position relative to the car.

The remaining (non-ground) points are grouped into **clusters** that likely correspond to cones.

Algorithm outline:
1. Pick a random unclustered point.
2. Find all nearby points within:
   - 0.5 m in **XY**
   - 4 m in **XYZ**
3. Remove those from the pool and treat them as a cluster.
4. Compute height = `max(z) - min(z)`.
5. Accept only clusters with `0.1 m < height < 0.5 m`.

Each accepted cluster stores:
```
[cluster_points, centroid_x, centroid_y, centroid_z]
```

**Outputs:**
- `/cluster/filtered_point_cloud` – all non-ground points.
- `/cluster/cluster_cloud` – only clustered (cone candidate) points.

---

### 5. Cone Color Estimation

The system can attempt to classify the color of each cone using two different strategies:

1. **Intensity-Based Method** – Estimates color from how reflective the cone’s surface appears in the LiDAR intensity readings.  
   Brighter or darker bands correspond to blue, yellow, or orange cones.

2. **Geometric Method** – Assigns cone colors based on their position on the track, assuming a standard race track layout where left cones are yellow and right cones are blue.

At this stage, cone color estimation is optional and primarily used for analysis or visualization.

Two strategies exist. Only one may be active.

#### (a) Intensity-based (`find_color`)
- Sort points by height.
- Inspect mid-height intensity band:
  - Low intensity → **Yellow cone**
  - High intensity → **Blue cone**
- Taller clusters (> 0.3 m) → **Orange cone**

#### (b) Geometric pattern (`cone_color`)
- Alternates left/right color assignment based on cone `y` position.
- Useful for race tracks where left cones are yellow and right cones are blue.

!!! note
    Currently, color classification is computed but not yet pushed into specific ROS color arrays (all cones go to `unknown_color_cones`).

!!! warning
    (b) will work good on straight track lines/rows of cones but when it goes into turns, one side of cones can be split into two colors.

---

### 6. Cone Publishing

Once cones are identified and their centroids calculated, they are packaged into a ROS message and published.  
Each cone entry includes its x–y position and a placeholder covariance value indicating uncertainty.

All cones are currently placed in a general “unknown” list, but future versions will separate blue, yellow, and orange cones into their own arrays.

This message can then be used by other parts of the system, such as path planning, to understand the layout of the track.


All clustered cones are packaged into a `ConeArrayWithCovariance` message:

```python
msg = ConeArrayWithCovariance()
msg.header.frame_id = "velodyne"
for cluster in clusters:
    point = ConeWithCovariance()
    point.point.x, point.point.y = cluster[1:3]
    msg.unknown_color_cones.append(point)
self.cone_pub.publish(msg)
```

**Each cone includes:**
- `x, y`: centroid in meters
- `covariance`: currently `[0, 0, 0, 0]`
- `color`: not filled (future use)

---

## Timing & Performance Logging

The node records how long each stage of the process takes — from reading the LiDAR data to publishing the final cone list.  
This helps identify bottlenecks and optimize performance.

Typically, a full LiDAR processing cycle completes within a few tens of milliseconds, making it suitable for real-time operation.

Each LiDAR callback prints a breakdown of computation time:

```
Timings
Read        : 0.003
Plane fit   : 0.010
Removal     : 0.004
Clustering  : 0.012
Color       : 0.002
Publish     : 0.001
Total       : 0.032 s
Clusters    : 14
```

This helps debug latency bottlenecks (e.g., large clouds, slow plane filtering).

---

## Coordinate System

All computations use the standard **Velodyne coordinate frame**, where:

- **X-axis** points forward,
- **Y-axis** points to the left,
- **Z-axis** points upward.

This consistency ensures that the cone positions are correctly aligned with other vehicle systems such as state estimation and planning.

| Axis | Direction | Description |
|------|------------|-------------|
| **X** | Forward | Vehicle’s forward motion |
| **Y** | Left | Lateral direction (positive left) |
| **Z** | Up | Height above ground |

All processing assumes this **Velodyne-style right-handed frame**.

!!!note
    Make sure to double check the coordinate system as it may vary across different departments and functions.

---

## Debug Topics Summary

Several debug point clouds are published for visualization in **RViz**, allowing developers to see each processing stage in real time:

- The filtered cloud (after removing unwanted regions)
- The non-ground cloud (after ground removal)
- The clustered cloud (showing cone candidates)
- The final cone detections

This layered approach helps in verifying each stage independently and diagnosing potential issues.

| Topic | Description |
|--------|-------------|
| `/cluster/ransac_point_cloud` | Raw ground ROI before plane segmentation |
| `/cluster/filtered_point_cloud` | Points after ground removal |
| `/cluster/cluster_cloud` | Only clustered cone candidates |
| `/cluster/cluster_point_cloud` | Unmodified input cloud |
| `/cones/lidar` | Final cone centroids |

---

## Optional Service Interface

The node includes an optional service interface that can send cluster data to another ROS node.  
This allows external modules to receive detailed segmentation masks or point data for further processing.

Although this feature is currently disabled, it provides flexibility for future extensions such as machine learning–based segmentation.

A prepared service client (`/cluster`) can serialize clusters into ROS `Image` messages for segmentation masks and coordinate triplets.  
It’s currently unused but allows external perception modules to receive labeled cluster data.

```python
self.cluster_service_client = self.create_client(_Cluster, '/cluster')
```

---

## Example Visualization Topics

You can view outputs in RViz:

```bash
rviz2
```

!!!note
    Make sure to source the environment.
---

Enable:
- `/cluster/filtered_point_cloud`
- `/cluster/cluster_cloud`
- `/cones/lidar`

The cones should appear as colored centroids, while point clouds show filtered and clustered layers.

---

## Known Limitations

While effective, the current implementation has several limitations:

- It requires a precomputed ground plane model — it does not estimate the plane by itself.  
- Cone color estimation is simplified but hardcoded, need to either do blind path planning or figure out how to effectively use intensity as weather and lightning conditions can affect it.  
- Clusters are processed per frame with no temporal smoothing.  
- Very close or overlapping cones might sometimes be detected as a single cluster.

Despite these, the pipeline provides a strong foundation for autonomous track perception using LiDAR only.

---

## Summary of Flow

```mermaid
flowchart TD
    A[LiDAR /velodyne_points] --> B[ROI Filter]
    B --> C[Ground Plane Removal<br>(via /cluster/plane_model)]
    C --> D[Cluster Non-Ground Points]
    D --> E[Color Classification<br>(optional)]
    E --> F[Publish ConeArrayWithCovariance<br>/cones/lidar]
    C --> G[/cluster/filtered_point_cloud]
    D --> H[/cluster/cluster_cloud]
    A --> I[/cluster/ransac_point_cloud]
```

---

## Future Improvements

- Integrate **plane detection via RANSAC** if no `/cluster/plane_model` is available.  
- Extend **color assignment** to fill `blue_cones`, `yellow_cones`, and `orange_cones` arrays.  
- Add **DBSCAN** or **Euclidean clustering** for improved robustness.  
- Fuse with **camera-based** color detection via `CAMERA_HEIGHT` parameter.
- **Subject to Change**
---

## Summary

The LiDAR-only pipeline transforms raw 3D point clouds into usable cone detections by:

1. Filtering the LiDAR data spatially.  
2. Removing the ground plane using a provided model.  
3. Clustering remaining points into cone-sized groups.  
4. Estimating cone color.  
5. Publishing their locations for autonomous navigation.

---
