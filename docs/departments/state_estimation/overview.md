# State Estimation

Put all documentation here!

# State estimation

State estimation is a part of autonomous that is in charge of communicating information throughout the system. There are approximately 4 key parts, the state machine, SLAM (Simultaneous Localization And Mapping), the EKF (Extended Kalman Filter) and the GPS (Global positioning system) research project. 

The state machine is in charge of communicating with the machine through ROS the current state that the car is in, if it is driving, stopped, if it is turning, etc. SLAM is implemented in order to be able to figure out where the car is relative to cones and the track. The EKF is effectively a machine learning software that learns from previous inputs what the hypothetical input should be to eliminate outliers. GPS is a research project that we hope to incorporate into our final solution. 

# Goals

Our general goal for the year is to improve our solution, understand previously written code about the EKF and SLAm and incorporate the GPS into our final solution. The primary goal with all of this is to document it properly so that the knowledge can properly be passed down to the new recruits. 

# Previous Implementations

## 2023/2024

Our state estimation system and SLAM was greatly improved upon. We bought a GPS and began some testing to see what solutions are possible to be integrated. 

### GitHub Code 🤓

* Fast SLAM: <https://github.com/FT-Autonomous/FT-FastSLAM-2023/blob/main/src/slam/fastslam/fastslam.py>
* SLAM node: <https://github.com/FT-Autonomous/FT-FastSLAM-2023/blob/main/src/slam/slam/slam_node.py>
* Extended Kalman Filter: <https://github.com/FT-Autonomous/ft_system/tree/master/src/ft_sensors>
* State Machine: <https://github.com/FT-Autonomous/ft_system/tree/master/src/ft_state_machine>

### Research Papers 📜 

### GitHub Code 🤓

* Fast SLAM: <https://github.com/FT-Autonomous/FT-FastSLAM-2023/blob/main/src/slam/fastslam/fastslam.py>
* SLAM node: <https://github.com/FT-Autonomous/FT-FastSLAM-2023/blob/main/src/slam/slam/slam_node.py>
* Extended Kalman Filter: <https://github.com/FT-Autonomous/ft_system/tree/master/src/ft_sensors>
* State Machine: <https://github.com/FT-Autonomous/ft_system/tree/master/src/ft_state_machine>

### Research Papers 📜 

* AMZ Driverless: <https://arxiv.org/pdf/1905.05150.pdf>