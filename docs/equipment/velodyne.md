# The Velodyne VLP16 (some changes)

![velo](https://ouster.imgix.net/OUSTER_product1-Post.jpg?ixlib=js-3.8.0&q=75&auto=format%2Ccompress&w=1200&max-w=2048)

The VLP 16 is our main LiDAR sensor. It's a 16 line LiDAR, hence its name.

Here are some important links:

- [VLP16 main page](https://ouster.com/products/hardware/vlp-16) - Propaganda information.
- [VLP16 datasheets and manuals](https://ouster.com/downloads/velodyne-downloads)
- [VLP16 ROS2 Driver GitHub repository](https://github.com/ros-drivers/velodyne/)

### Using the Velodyne

You'll need to download/clone the velodyne ROS2 drivers from [the "ros-drivers/velodyne" github repo](https://github.com/ros-drivers/velodyne/). You compile it like you would compile any other ROS2 package. After cloning the repo, make sure you are on the correct branch of the repo. Depending on what installation of ROS2 you have, you may need to switch to the correct branch. For ROS2 galactic, the `04a7ca722eaed13fe902366e0831717e91f9f77a` commit has been conformed to work. If you are using ROS2 humble, you dont have to switch branches. Then you can compile the package with the following commands:

```
# Switch ros2 to the branch you want to clone if necessary
git clone https://github.com/ros-drivers/velodyne/ -b ros2

# Then you have to source the environment
. /opt/ros/humble/setup.bash

# Then you can build the package
colcon build --symlink-install

# Source the workspace
. install/setup.bash
```

Power the LiDAR using the DC jack and connect the LiDAR to the host computer using ethernet.

#### Setting up the Network

Note that this section was derived from the manual. If anything here doesn't work, refer to the manual for further clarification.

To set up the network, run the following command in the terminal to start the NetworkManager connection editor.

```
nm-connection-editor
```

There may already be a "Velodyne" ethernet connection in NetworkManager, if so, skip to the activation step.

If not, you will need to create a new connection, by clicking the `+` button in the bottom left corner. Select `Ethernet` from the dropdown menu and click `Create`. Name the connection as "Velodyne" and then navigate to the "IPv4 Settings" tab. Now we need to add the addresses for it by clicking `Add`. Give the velodyne static ip as `192.168.1.200`. Set the subnet mask as `255.255.255.0` and the gateway as `255.255.255.255`.

#### Activating the Velodyne Network

To activate the Velodyne network configuration, run the command.

```
nmcli connection up Velodyne
```

Run the command `ip -br a` to verify that the network interface corresponding to the velodyne is `UP` and it should be color coded green.

#### Configuring the Velodyne via the Web Console

Once the network has been set up, the Velodyne can be configured in a web console made available by typing the IP address `192.168.1.201` on a web browser. (yes, this is different from the IP address assigned to the network interface).

#### Configuring the Velodyne via ROS2 Parameters

ROS2 configuration parameters are stored in the file `velodyne/velodyne_driver/config/VLP16-velodyne_driver_node-params.yaml`.

##### Increasing the Publish Rate of the LiDAR

You can tweak the `rpm` parameter to `1200.0` to get the LiDAR to publish messages at 20hz instead of the default 10hz.

#### Troubleshooting

##### Power

First, verify that the velodyne is actually turned on. You should hear a dim continuous hum from the velodyne coming from the internal motor that spins the laser. Additionally, when you place your hand on the velodyne you should feel a continous vibration.

You may run into a situation where the green power indicator on the VLP-16 is on but the device is not spinning. One possible cause is an unstable mounting platform. If the VLP-16 tries to spin up the motors and in doing so rotates the chassis of the sensor itself multiple times, it will not start up for mechanical safety reasons.

##### Networking

If the network interface is down most of the time but it is spinning, you may have a hardware fault. One of the most frequent issues we have with out LiDAR is that the wires inside the interface box come loose. You can open the interface box and check the wires to see if they are connected properly. if they are not connected properly, you can try reconnecting them. Make sure the power is off before you do this!.

#### Running the Velodyne

Once the network is set up, you can run the velodyne driver with the following command:

```	
ros2 launch velodyne velodyne-all-nodes-VLP16-composed-launch.py
```

#### Visualising the LiDAR Output

Source your ROS workspace and then run the following:

```
rviz2
```

You will need to add the topic corresponding to the point cloud. To do this, click the `Add` button in the bottom left hand side of the screen. In the main menu you should see the `PointCloud2` topic. You want to add that to the view. Then expand the new `PointCloud2` menu in the menu in the left hand side. You need to configure the topic to point to the name of the topic published by the driver. In this case the topic name is `/velodyne_points`. Finally, set the `Fixed Frame` option in the global options menu to `velodyne`. Once this is done you should be able to see everything in RVIZ (see more on how to use RVIZ2 here).

See also:

- [Our TF2 article](../resources/tf2.md) - To understand more in depth what is meant by a "frame".
- [Our RVIZ article](../resources/rviz2.md) - To learn more about RVIZ2.