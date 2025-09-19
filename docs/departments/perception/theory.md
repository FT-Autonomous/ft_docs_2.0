# Perception Theory
- This section covers the theoretical aspects of the Perception department.
- Detailed explanations of algorithms and methodologies used.
- Research papers and references.

### Introuction to Perception
At the heart of any autonomous vehicle system, the Perception module enables it to understand and interact with its environment. In the context of FSAI, where the primary task is to navigate a track delineated by cones, the Perception module plays a critical role, responsible for detecting, classifying, and localizing cones with high accuracy and efficiency, providing essential data for subsequent modules like State Estimation, Path Planning, and Control.

The Perception module processes raw sensor data to generate a coherent representation of the environment, allowing the vehicle to interpret its surroundings in real time. In FSAI, this means identifying cones on the track, determining their color, and pinpointing their 3D positions relative to the vehicle.


<!-- ![Cone map](media/wlco.png) -->


### Perception Workflow in FSAI
The Perception module integrates data from multiple sensors to achieve its objectives. The primary sensors used are 
- LiDAR : Captures 3D point cloud data to provide precise spatial infromation about its surroundings.
- Cameras : Capture 2d images that provide rich color and texture information, essential for classifying cone colors.
These sensors complement each other, with LiDAR offering accurate depth perception and spatial data, while cameras provide detailed color and visual context. By fusing these data sources, the Perception module can achieve robust and reliable results

### Objectives of the Perception Module
The primary objectives of the Perception module in FSAI are:
- Cone Detection: Identify cones on the track using data from LiDAR and cameras.
- Cone Classification: Determine the color of each cone, which is critical for understanding the track's layout.
- Cone Localization: Calculate the precise 3D position of each cone relative to the vehicle.
- Real-Time Perormance: Process data quickly to ensure the cehicel can react to dynamic changes in the environment.