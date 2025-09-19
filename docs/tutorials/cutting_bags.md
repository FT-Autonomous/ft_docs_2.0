# Cutting Bags Down to Size

The bags that we recorded at competition were massive and measured around 22GB in size.

Some of this size is inevitable, but some of it could realistically be avoided. The two main contributors to the massive size of the bags are the LiDAR point cloud data and the video stream data. The LiDAR point cloud data is by nature noisy positional data that. Even if we were to attempt to compress such data, we would likely not see any significant results, thus, the bloat caused by the LiDAR point cloud data can be considered necessary (Though one could imagine that using 64 bit floating point numbers is redundant and we could save space by better quantising LiDAR point cloud readings. This would be a complex operation). The second contributor, video stream data contributes bloat due to a technical issue with the way they are stored. On track, the camera publishes singular snapshots of what it can see in the form of uncompressed RGB pixel arrays. When we record data, we use the ROS2 bag mechanism which simply papers over the exact type of data being recorded and stores it directly to a file without any further optimisation. Do you see the problem here? Storing videos as sequences of uncompressed pixel arrays is just about one of the least efficient things you can do (a three minute 1280x720 video at 15fps ends up being something like 7GB!). Ideally we would want to switch to a recording system that uses companion video files that use standard compression techniques in mp4 containers and the like but we at this time do not know how best to do that.

An ad hoc solution to dealing with such massive bags is to cut them down to a more manageable size that focuses only on the part of the data that we are interested in. In this particular case, we are talking about a 22GB bag file where the starting few seconds records us moving the vehicle onto the track and the latter few seconds consist of our pit stop team moving the car off the track again. All that can be cut out if we just want to test our algorithms.

A naive way to cut down ROS2 bags might be to play the bags on the network using the `ros2 bag play <bag>` command, and then subsequently record the relevant section of the bag using the `ros2 bag record <bag>` command. However, there's a much better way of doing this.

There is an open source tool that we can avail of to cut our bags down to size present in the `ros2bag_tools` utility ([github](https://github.com/AIT-Assistive-Autonomous-Systems/ros2bag_tools)). It can be thought of as an extension to the existing `ros2 bag …` family of commands. The way it's "installed" is by cloning the repository into a workspace and then compiling and sourcing that workspace. Here are the relevant commands one might do:

```
cd ~/ft
git clone https://github.com/AIT-Assistive-Autonomous-Systems/ros2bag_tools
colcon build --packages-select ros2bag_tools
```

Our subject bag in this case is called `second_trackd` (don't ask why) stored in `~cleavon/Desktop/Bags/second_trackd`. To play that bag, you can use the command:

```
ros2 bag play ~cleavon/Desktop/Bags/second_trackd
```

To cut the bag use the following command:

```
ros2 bag cut ~cleavon/Desktop/Bags/second_trackd --start 60 --duration 180 --output second_trackd_cut
```

To visualize the bag, run `rviz2` in a new terminal and add all appropriate topics (see [The ROS2 Visualisation Stack](https://docs.formulatrinity.ie/rviz/))

If none of the visual feeds show up make sure you change the camera to "odom" and run the following command in a different terminal for tf2 reasons:

```
ros2 run ft_system pose_transform_publisher
```