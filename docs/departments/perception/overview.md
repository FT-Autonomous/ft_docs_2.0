# Overview 

- Perception is at the heart of any autonomous system. Before the car can decide where to go, it needs to see the world around it. The perception team aims to do just that. Through a variety of sensors and ingenious algorithms, we allow the car to see and understand the world around it.

## Previous Implementations

### 2021/2022

- **Sensor Inputs**: ZED2 Camera
- This was our first year competing at FSUK. The approach was based on the RGB + XYZ (color + position / depth) point cloud data that could be obtained from the ZED2 camera. The idea was that we would perform some kind of segmentation of the pixels in the image corresponding to cones. segment them into clusters corresponding to cones, and then average the position values of cluster to obtain a mean position value for the cone.
- This code is all stored in the [fta_perception_ros](https://github.com/FT-Autonomous/fta_perception_ros/) repository. Today, most of it is deprecated and is only useful in the sense that you can see some examples of how to use the stereolabs SDK. The only part that is kind of still in use is [ft_cluster_cpp](https://github.com/FT-Autonomous/fta_perception_ros/blob/main/src/ft_cluster_cpp/). This is a GPU accelerated clustering algorithm that can be used to divide an unordered array of points into discrete sub arrays based on pairwise distances between points (you can read about clustering algorithms [here](https://machinelearningmastery.com/clustering-algorithms-with-python/)).

#### 2022/2023

- **Sensor Inputs**: Livox Avia LiDAR, ZED2(which we didn’t use)
- Perception first begins by taking in data from the LiDAR sensors, which is utilised segmenting the cones. Once the data arrives, the code opens the bag file and starts processing. Next part of the processing was to remove the ground and sky data points.  By using some thresholds and conditions and by using grids and creating cells in them, the ground and sky points were removed.
- After we do geometrical filtering, the clustering algorithm takes place in which the cones are clustered out using their known dimensions i.e. their width and height. After clustering, by their positions we are able to assign them colour since it is already known that cones on the left are blue and cones on the right are yellow.
- Some drawbacks: The lidar we have only had a 70 degree angle which meant that during turns only one row of cones could be seen which also meant that the colour would be wrong as it depends on the coordinates of the cone and assigns the colour.

#### 2023/2024

- **Sensor Inputs**: Velodyne VLP16 LiDAR, ZED2
- The input from the LiDAR is in the form of a tagged point cloud.
- The input from the ZED camera is in the form of an RGBD bitmap image. The image will be passed to a YOLOv5 model that was trained on the FSOCO dataset. While YOLOv5 isn't the newest model available, we decided that the wealth of information available on the model would be far greater than any minor speed improvements we achieve by using a newer model. In our testing, the model consistently achieved 80% accuracy in good lighting conditions and around 50-60% accuracy in poor lighting.
- Once we compute the bounding boxes and colours of the cones using the YOLO model, we combine this with the LiDAR data in a sensor fusion approach to segment only the points that are on the cone. The points in this location can then be averaged to give us the center of the cone. This data, along with the colours of the cones will then be published on a ROS topic for the other systems to use.


### Resources

- [CS231n Intro to Convolutional Neural Networks](https://cs231n.github.io/) — This is a great resource to use to get an understanding of how machine learning works.
- [Stanford Notes on Camera Models](https://github.com/braca51e/cs231a-1/blob/master/Lecture%20Notes/01-camera-models.pdf) — Useful notes on understanding how certain matrix multiplication operations used for camera calibration work.