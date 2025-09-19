
# A Guide to TF2 (The ROS2 Transforms Library)

In robotics, we often deal with a number of different perspectives of the world from different vantage points at different points in time. In the case of a humanoid robot, there is naturally the vantage point of the robot from the cameras on it's head. There's also the global vantage point of where the robot thinks it is in the world. In the case of a Formula Student AI vehicle, the relevant vantage points include the point of view of the world from the LiDAR, the point of view of the world from the camera and also a conception of where the car is in the world at large. We call these vantage points coordinate frames.

Coordinate frames can generally be related to each other. In ROS2, we consider coordinate frame relationships to be related by transform tress. For example, our vehicle has a location and a rotation relative to an origin the world, and likewise our camera has a location and a rotation relative to some origin point on the vehicle. The combination of this location and rotation is denoted as a transform. The ROS2 Transforms Library (called TF2) allows us to define the transformation trees that relate our coordinate frames so that different parts of the robotics stack can transform observations and general data points from one coordinate frame to another in a standard way.

## Debugging TF2 Issues

Some are lucky in life and never have to debug TF2 issues. Others do not have that good fortune.

### Exporting The Tree to a PDF

```
ros2 run tf2_tools view_frames
```

You can use this output to diagnose cases where your transforms tree is disconnected in cases where it shouldn't be.

One particular example where this arises is when using open source libraries for sensors which export sensor internal transformation tree information. The ZED2 SDK is an example of this, which defines the coordinate frames for the left camera, the right camera and even for things such as the magnetometer and the barometer, and also defines the relationship between those coordinate frames and the base of the ZED2 camera. The base of the ZED2 camera is then linked to the base of some parent body, which the ZED2 SDK by default assumes is named `base_link`. This may not be what you have actually named the parent body of your ZED2 camera, for example, in our code base, across different eras, we have used the frames `base_footprint` (taken from the name given to the vehicle base in EUFS) to `chassis` (a more natural sounding one, also taken from EUFS).

### Getting a "Live" View of the Transforms Tree

{% image src="http://wiki.ros.org/rqt_tf_tree?action=AttachFile&do=get&target=snap_rqt_tf_tree.png" width="400px" /%}

The material produced by the `view_frames` utility is useful but invoking that program becomes cumbersome. You can use the `rqt_tf_tree` utility to make this a little less bothersome. You can install it using:

```
sudo apt install ros-humble-rqt-tf-tree
```

You can then run it via:

```
ros2 run rqt_tf_tree rqt_tf_tree
```

## TF2 Only Matches Transforms to Exact Timestamps

Consider a scenario where we have a coordinate frame named `odom` which is anchored at some point on the race track. For simplicity, we can consider this to be the start of the circuit. No imagine a scenario where at time step `t`, we have said that our vehicle (represented by the `chassis` frame) is 5 meters away from the origin at an angle of 45 degrees. Now, let's say at time step `t + 0.0001`, we want to know the transform from the `odom` frame to the `chassis` frame. Does it make sense to simply use the previous transform, since it's within some threshold?

TF2 would beg to differ. In TF2, we explicitly **cannot** use transforms on data unless the transform and the data have exactly the same timestamp.

### Why This Makes Sense

This is possibly related to how you would certainly not want to make a predefined threshold based on something like a human intuition of when two events are close to each other, seeing as ROS2 topics and transforms allow time stamping at a precision down to the nanosecond. For robotics operating on small timescales (possibly using real time operating systems), interpolation on a human scale would be incoherent and cause chaos. One might then ask why ROS2 doesn't let us just specify the threshold ourselves. Not actually sure about this &#x2013; perhaps it would become too complex having to specify interpolation thresholds for each different kind of data. The implication of all this is once again that we have to make sure that all data points (such as point cloud data or camera data) that we want to transform have an associated transform tree whose time stamp is equal to the time stamp in that data point. The community probably deemed this to be acceptable at some point.

Not fully understanding the issue of time has caused us a world of pain trying to get RVIZ to render topics. Once you understand how TF2 handles time, and your TF2 tree looks about right, you should not face any issues with TF2 at all.

## Transform Publishers

We rig our own transform tree (though there is probably a package out there that can do it for us).

Static transform publishers are used to denote aspects of the transform tree that always stay the same, such as between two parts of a (very) rigid body. On the ZED2 camera, the transform from the ZED2 base frame to the ZED2 left camera frame is usually done using a static transform.

Dynamic transform publishers are used to denote aspects of the transform tree that change with time. In the case of our code, the main dynamic transform is the `odom` to `chassis` transform frame which describes where the vehicle is relative to starting location on the track using information gleaned from state estimation. You can run teh node that publishes these transforms as follows.

```
ros2 run ft_system pose_transform_publisher
```
