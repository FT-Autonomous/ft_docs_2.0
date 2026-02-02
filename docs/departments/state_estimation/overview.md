# State Estimation

State Estimation is in charge of two main parts: State Machine and SLAM (Simultaneous Localization and Mapping). Since, SLAM relies on sensor inputs, this necessitates the use of an EKF (Extended Kalman Filter) and a GPS (Global Positioning System).

The state machine is responsible for communicating the vehicle's current state through ROS, such as whether the car is driving, stopped, or turning. SLAM is used to make a map of the track and determine the car's current position relative to the track. The EKF is used to estimate the car's current position.

# Goals
1. Implement GraphSLAM
2. Use the GPS as an input for GraphSLAM
3. Rewrite the logic for the State Machine

# Previous Implementations

## 2025/2026

In the 2025/2026 iteration, a new state machine was introduced to improve system understanding. GraphSLAM was used, providing a more robust and globally consistent solution compared to previous approaches. Additionally, GPS has been incorporated to enhance global position estimation and long-term accuracy. The GPS has it's own built-in EKF. This simplifies State Estimation stack.

## Up to 2024/2025

In earlier implementations up to the 2024/2025 development cycle, the system primarily relied on FastSLAM for simultaneous localization and mapping. Sensor data from the ZED camera and LiDAR were fused using an Extended Kalman Filter (EKF) to estimate the vehicle's pose. This approach enabled the vehicle to localize itself within the environment while constructing a map of surrounding features. The EKF played a key role in reducing sensor noise and improving pose accuracy by combining visual and LiDAR measurements. While effective, this implementation had limitations in robustness, as our algorithm would consistently alter the cone's positions due to noise.

See more:

- FastSLAM: <https://github.com/FT-Autonomous/FT-FSAI-23>
- State Machine and EKF: <https://github.com/FT-Autonomous/ft_system>
