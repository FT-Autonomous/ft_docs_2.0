# The ZED2

The ZED2 is the main camera we use. In addition to being a camera, it has sensors as an IMU, gyroscope, magnetometer and other sensors. The ZED2 is also capable of producing a pose estimate of where it is in the world using the aforementioned sensors.

![zed2](https://www.stereolabs.com/docs/api/ZED-2-front.webp)

Some links:

- [ZED2 schematic](https://static.generation-robots.com/media/zed-2i-technical-drawings.pdf) - Technical drawings of the ZED2.
- [ZED2 homepage](https://www.stereolabs.com/en-ie/products/zed-2) - Pricing and propaganda information.
- [ZED2 datasheet](https://www.generationrobots.com/media/zed2-camera-datasheet.pdf) - Outlines a number of the ZED2 specs.

The SDK you use to interface directly with the ZED2 is called the stereolabs SDK.

- [Stereolabs Python SDK](https://www.stereolabs.com/docs/api/python)
- [Stereolabs C++ SDK](https://www.stereolabs.com/docs/api)

### Known Issues with the ZED2

#### USB Connecton Issues

Modern ZED camera models have removable USB C plugs. The ZED1 and ZED2 do not have this kind of plug which means that the connection extremely flaky as the connector is constantly bending when mounting / unmounting / motion and it cannot be removed. The effects of this include:

- Camera rebooting when the cable is touched / requring a very specific and static wire orientation to get anything to work.
- The camera staying on for a few seconds before rebooting and not recovering from the corruption that required the reboot.

If you want to invest in a new ZED camera, invest in a newer ZED model such as the ZED2i that has a removable USB C cable.

##### Resolving USB Connection Issues

The only solution seems to be to follow [this instructables article](https://www.instructables.com/ZED-Disassembly/).

### Using the ZED2

#### Compiling the ZED ROS2 Wrapper

The ZED ROS2 Wrapper is a suite of packages that make the sensors on the ZED2 available through a ROS2 interface. The following is a guide on how to compile these packages. The wrapper requires an NVIDIA GPU to operate -- the following is not applicable to platforms that don't have an NVIDIA GPU or whose drivers aren't working.

If you are using a shared platform such as the Jetson somebody has probably compiled the wrapper already. In this case you can skip to the sourcing step. Otherwise you can continue.

As prerequisites, in addition `ros-humble desktop`, you must install the fllowing packages:

```
ros-humble-nmea-msgs ros-humble-geographic-msgs ros-humble-diagnostic-updater ros-humble-robot-localization ros-humble-xacro ros-humble-zed-msgs
```

Clone the github repository [ZED2 ROS Wrapper](https://github.com/stereolabs/zed-ros2-wrapper) and the [ZED2 Interfaces Repo](https://github.com/stereolabs/zed-ros2-interfaces):

```
git clone --recurse-submodules https://github.com/stereolabs/zed-ros2-wrapper
git -C zed-ros2-wrapper checkout f88bb492fa01754365f0ac0d56e70fd29d55bbee
git clone --recurse-submodules https://github.com/stereolabs/zed-ros2-interfaces
git -C zed-ros2-interfaces checkout 417073fce8adf398f7295ec25148a046aeb0eed6
```

**Note**: If you are still using ROS2 galactic, comment out the lines in `zed_camera_component.cpp` that trigger an error if the ROS version is not `IRON` or `HUMBLE` or `FOXY`. As of \[2024-01-21 Sun\], the main branch on github compiles fine under galactic. Otherwise, you can ignore this disclaimer.

Navigate to the `zed-ros2-wrapper` folder and build:

```
. /opt/ros/humble/setup.bash
colcon build --symlink-install
. install/setup.bash
```

#### Ensuring You Have Access to the Camera Device

First you need to ensure that you have access to the camera device. On linux, cameras are represented on the filesystem through the files `/dev/videoN` (where `N` is a number). By default, these are owned by the `root` user and the `video` group. You can check what groups your current user is in using the `groups` command. If you are not in the `video` group, add yourself to the group using the following command, replacing `USER` with your username.

```
sudo usermod -aG video USER
```

Another quick fix to this issue is simply changing the permissions on the video files such that everybody can access those files:

```
sudo chmod a+rw /dev/video*
```

#### Running the ZED2 ROS wrapper

After sourcing, you can run the wrapper as follows:

```
ros2 launch zed_wrapper zed2.launch.py camera_model:=zed2
```

Use `ros2 topic list` to explore the topics being published and use `ros2 topic info TOPIC` to view information about the topic types. To view information about a ROS2 topic type, use `ros2 interface show INTERFACE`.

##### Visualising the Camera Output

{% image src="https://github.com/stereolabs/zed-ros2-wrapper/raw/master/images/sim_rviz.jpg" width="450px" /%}

Source your ROS workspace. Then download [this example RVIZ config](https://github.com/stereolabs/zed-ros2-examples/blob/master/zed_display_rviz2/rviz2/zed_stereo.rviz) and then run the following:

```
rviz2 -d zed_stereo.rviz
```

See also:

- [Our TF2 article](../resources/tf2.md) - To understand more in depth what is meant by a "frame".
- [Our RVIZ article](../resources/rviz2.md) - To learn more about RVIZ2.

##### Multi Camera Setup

the `zed2.launch.py` script takes as arguments `camera_name` and `serial_number` which are relevant in a multi camera setup.

```
'camera_name':
        The name of the camera. It can be different from the camera model and it
        will be used as node `namespace`.
        (default: 'zed')
```

```
'serial_number':
        The serial number of the camera to be opened. It is mandatory to use this
        parameter in multi-camera rigs to distinguish between different cameras.
        (default: '0')
```

The zed2 with the sticker has serial number is `25382294` and the zed2 without a sticker is `23038477`. So essentially, in two separate terminals, you execute the zed ros2 wrapper as:

```
ros2 launch zed_wrapper zed2.launch.py camera_name=zedA serial_number:=25382294
```

and

```
ros2 launch zed_wrapper zed2.launch.py camera_name=zedB serial_number:=23038477
```