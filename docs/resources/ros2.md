# Introduction

[What Is ROS2? - Framework Overview](https://www.youtube.com/watch?v=7TVWlADXwRw).

The YouTube video above gives a great overview of ROS 2 features & functionality. Use the ROS 2 galactic documentation in the next section to practice using ROS 2 in either Python / C++.

## Link to Official ROS 2 Humble Documentation

Documentation home page [ROS 2 Documentation: Humble documentation](https://docs.ros.org/en/humble/index.html).

Link to Installation Instructions [Installation — ROS 2 Documentation: Humble documentation](https://docs.ros.org/en/humble/Installation.html).

Link to the Tutorials [Tutorials — ROS 2 Documentation: Humble documentation](https://docs.ros.org/en/humble/Tutorials.html).

## Building ROS2 Packages

It's *essential* that you understand the concept of workspaces, underlays and overlays.

- Read about these concepts [on the official ROS2 wiki](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#basics).

The command `colcon build` is used to build colcon packages. *You should always be concious of the workspace folder you're building in*. Generally this will be the `~/ft/` folder unless you are explicitly doing some experimental work elsewhere.

When you run `colcon build`, the ROS2 build system will traverse the file system recursively, find all packages, and compile all packages that have changed or have not been compiled. Packages are identified by folders that contain a `package.xml` file. Nested packaging is not allowed, i.e. packages cannot contain sub-packages.

```
. /opt/ros/humble/setup.bash
colcon build --symlink-install
. install/setup.bash
```

### Building Specific Packages

The `--packages-up-to` argument selects a package build, and also informs colcon to build all of the packages that it depends on. It can be used as follows:

```
colcon build --symlink-install --packages-up-to PACKAGE_NAME
```

Other pacakge selection commands are documented in [the colcon docs](https://colcon.readthedocs.io/en/released/reference/package-selection-arguments.html).

### Exporting Python Packages

If you structure your python project as follows:

```
📄 setup.py
📂 ft_module
└📄 __init__.py
└📂 sub_module.py
 └📄 __init__.py
 └📄 sub_sub_module.py
```

When you build your package using colcon and attempt to execute the following python code, you may get a `ModuleNotFound` error:

```
import ft_module.sub_module.sub_sub_module
```

The reason behind this is that by default, ROS2 configures the project to only export the top level package. The culprit is in the `setup(packages=[...])` component of the `setup.py` script. In our case, this section might look something like this:

```python
setup(packages=["ft_module"], ...)
```

The fix is to use the `setuptools.find_packages()` utility, wich will recursively look for all submodules as follows:

```python
from setuptools import setup, find_packages
...
setup(packages=find_packages(), ...)
```

## The ROS2 Command Line Interface (CLI)

Here are some useful ROS2 CLI commands.

- `ros2 topic list` … lists all the topics that are being subscribed to and published.
- `ros2 topic echo TOPIC` … displays the messages published to that topic.
- `ros2 topic hz TOPIC` … shows the publishing rate of the topic.
- `ros2 topic info TOPIC` … shows information about a topic.
- `ros2 bag play SOME_BAG` … play a bag.
- `ros2 bag play SOME_BAG --loop`  … if you want the bag to keep playing on loop.
- `ros2 run PACKAGE_NAME EXECUTABLE_NAME` … runs an executable from a package. For example, `ros2 run slam slam_node` launches the `slam_node` node in the slam package. The other node in the slam is the `plotter` node.
- `ros2 launch PACKAGE_NAME LAUNCH_FILE_NAME` … runs the specified launch file. For example, `ros2 launch slam slam.launch.py`.
- `ros2 run rqt_reconfigure rqt_reconfigure` … opens a Graphical User Interface (GUI) for dynamic reconfiguration of running ROS 2 nodes. This interface enables viewing and modifying node parameters in real time without restarting the nodes.
- `ros2 wtf` AKA `ros2 doctor` … shows warning from your system.
- `ros2 run rqt_graph rqt_graph` … generates a graph showing the topics and their subscription and publication connections.

### Extra

ROS by default publishes all messages to the network. Without any mechanism for segregating the network out into disjoint units, it would become crowded and isolated testing would become impossible. ROS provides the concept of "domains" to manage this. A domain is represented by a number or an index. Setting the domain scopes topics to that domain, meaning that subscribers will only be able to detect topics published by nodes in the same domain, allowing the network to be segregated.

Generally, you can just keep your domain as the default value, which is `0`. Unless you are explicitly doing advanced stuff over the network, ROS domains don't matter much.

Read more about domains here:

- [The ROS\_DOMAIN\_ID — ROS 2 Documentation](https://docs.ros.org/en/humble/Concepts/About-Domain-ID.html)

### ROS Services

[Understanding services — ROS 2 Documentation: Humble documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)  


## Other ROS2 Components

### TF2

TF2 is a ROS library for managing coordinate frames and translating between them.

- [About tf2 — ROS 2 Documentation](https://docs.ros.org/en/humble/Concepts/About-Tf2.html)
- [An article on tf2 in the context of Formula Trinity](../resources/tf2.md)

### RVIZ

RVIZ is a program used to visualise the state of a robotics system.

- [The official RVIZ2 article](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html)
- [An article on rviz2 in the context of Formula Trinity](../resources/rviz2.md)