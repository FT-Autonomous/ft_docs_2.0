# A Guide to RVIZ2

We use RVIZ2 to visualise our bags and the operation of our stack (some other teams such as EUFS have mostly migrated to using Foxglove Studio).

There are a number of types of data that can be visualised by RVIZ naively, such as:

The things that we visualise in RVIZ include:

- Point cloud data from our LiDAR
- Camera image data from our ZED2
- Pose information from state estimation
- … Probably more

Some data types have their presentation provided by external packages we use, namely, the `eufs_msgs` family of topics have their presentation provided by the `eufs_rviz_plugins` package. These include:

- Predicted cone locations from perception as an array of cones
- The map as currently conceived by state estimation as an array of cones
- The path planning trajectory as an array of cones.

Finally, some things we provide ourselves using RVIZ markers, including:

- Path planning internal state data
- Desired speed and steering angle from control
- Lookahead circle from control

You can run RVIZ2 with the default config using the command:

```
rviz2
```

Generally when you start RVIZ2, you manually add all of the topcis or data streams that you are interested in viewing.

This should only be done once. The data streams chosen to be displayed and their configuration on the screen in RVIZ2 can be saved to a configuration file and loaded at a later time. It is quite annoying to load in all of the topics you want to visualise each time you want to start RVIZ2 so it is recommended to actually save a file. You can do this via **File** / **Save As**. It's recommended to save regularly. You can save a couple microseconds of navigating to the configuration loading menu and launch RVIZ2 directly with the config file you are interested in using the command:

```
rviz2 -d ~/.rviz2/myConfig.rviz
```

The following outlines the steps one would take to visualise most of the topics that are relevant to understanding the stack.

- Set fixed frame to `odom` and set target frame to `odom` also.
- Add a `PointCloud2` on `/velodyne_points`. Set size to `0.05` squares.
- Add a `ConeArrayWithCovariance` on `/cones`.
- Add a `ConeArrayWithCovariance` on `/map`. Set color display mode to `gray`.
- Add a `WaypointsArrayStamped` on  `/trajectory`.
- Add a `Path` topic on `/zed/zed_node/path_odom`
- Add a `Camera` topic on `/zed/zed_node/left/image_rect_color`