# ROS Path Planning Tutorial

To follow this tutorial, first ensure that you are fully set up with our code, EUFS simulator and gazebo. There is a guide on how to do that [here](../tutorials/Getting_started.md)

### Analysing Topics

Once set up, you'll want to start the simulator as follows:

```
cd ~/ft
. install/setup.sh
ros2 launch eufs_launcher eufs_launcher.launch.py
```

With the simulator running, open another terminal and navigate to the `~/ft` folder. Use the following command to list what topics are being published and their associated types.

```bash
ros2 topic list -t 
```


{% figure src="https://docs.ros.org/en/jazzy/_images/Topic-MultiplePublisherandMultipleSubscriber.gif" height="200px" /%}

What is a ROS topic? ROS topics are like TV channels carrying a stream of messages. For path planning, you will listen on a stream of cone data and output a stream containing waypoints for the car to follow. If you look through the list that was printed after the list command, you will see a topic type called `/cones` and the type `eufs_msgs/msg/ConeArrayWithCovariance` next to it. To print information about the `ConeArrayWithCovariance` message, we can use the following command:

```bash
ros2 interface show eufs_msgs/msg/ConeArrayWithCovariance
```

### Making a ROS Node

With this information, we can start writing python code to interface with the ROS topics. The first step is to create a new ROS2 python project to store the code. This can be done using the following command (make sure that you run this in the \~/ft folder):

```bash
gh repo clone FT-Autonomous/ft_euclid ft_path_planning
cd ft_path_planning
```

This clones the path planning repository and switches to the ConeMatcher branch, which is the main one in use at present. An example of the most basic ROS2 node is the following, which should be placed in the `~/ft/ft_path_planning/ConeMatcher/ros.py` file.

```python
from rclpy.node import Node
import rclpy

class PathPlanningNode(Node):
    def __init__(self):
        super().__init__("path_planning_node")
        print("node running!")

def main():
    rclpy.init()
    rclpy.spin(PathPlanningNode())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

`Rclpy` stands for ROS Client for python. There is also `rclcpp`, for c++ masochists, which we are not. To make this accessible from ROS, we have to modify the `setup.py` file. Navigate to the `console_scripts` array and add the following entry:

```python
...
console_scripts = [
"ros = ConeMatcher.ros:main"
]
...
```

This will enable you to run the command `ros2 run ft_path_planning ros` in order to start the node. Before doing this, you need to compile the project again using the following command (make sure that you execute this in your workspace, which is probably \~/ft):

```bash
colcon build --symlink-install --packages-up-to ft_path_planning
```

Once you have done this, re-import ROS using the following command (this is necessary as we added new files as opposed to just modifying existing files):

```bash
. install/setup.bash
```

Then run the node using the following command.

```bash
ros2 run ft_path_planning ros
```

This node does absolutely nothing\! It is just meant to be used as a template in the next step. We will create a subscriber that listens to the `/cones` topic being published by the simulator. If you are not sure what it means for a node to publish a topic, refer to the graphic above or refer to the [section on understanding topics in the ROS documentation](https://docs.ros.org/en/galactic/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html). Here is the modified version of the `ros.py` script:

```python
from rclpy.node import Node
import rclpy
from eufs_msgs.msg import ConeArrayWithCovariance

class PathPlanningNode(Node):
    def __init__(self):
        super().__init__("path_planning_node")
        print("node running!")
        self.create_subscription(ConeArrayWithCovariance, "/cones", self.cones_cb, 1)

    def cones_cb(self, cones):
        breakpoint()

def main():
    rclpy.init()
    rclpy.spin(PathPlanningNode())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

Here is an explanation of the changes made to the file:

* We imported the ROS interface type used for cone arrays at the top level import statement (eufs\_msgs.msg.ConeArraywithCovariance).  
* We created a new subscriber and associated it with the callback `cones_cb`. Every time the simulator publishes more cones, the `cones_cb` function will be invoked. The function arguments are the topic type, the topic name, the topic callback function and a queue size ([API documentation here](https://docs.ros2.org/galactic/api/rclpy/api/node.html?highlight=create_subscription#rclpy.node.Node.create_subscription)).

After saving this file, run `ros2 run ft_path_planning ros` once more. Running it in the terminal should put you in the debugger with the prompt `(pdb)`. Enter the following commands to inspect the cones data structure around the breakpoint.

* `[(cone.point.x, cone.point.y) for cone in cones.blue_cones]` \- this command will extract the x, y coordinates for each blue cone.  
* `[(cone.point.x, cone.point.y) for cone in cones.yellow_cones]` \- the same for yellow cones.  
* `cones` \- should print out the ConeArrayWithCovariance data structure.

You can type in `exit()` to leave the debugger or use `Ctrl+D`.

### RVIZ Example

Place the following code in the `ros.py` file that you created in the path planning package:

```python
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from math import cos, sin
import time

class PathPlanningVisualiser(rclpy.node.Node):
    def __init__(self):
        super().__init__("path_planning_visualiser")
        self.marker_publisher = self.create_publisher(MarkerArray, "/marker", 1)
        self.timer = self.create_timer(0, self.timer_cb)

    def timer_cb(self):
        marker_array = MarkerArray()
        t = time.time()

        # create a text marker
        marker = Marker()
        marker.header.frame_id = "/base_footprint"
        marker.ns, marker.id = "ft_text", 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = f"It is: {t}"
        marker.scale.z = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 1.0, 1.0, 1.0
        marker_array.markers.append(marker)

        # create a line marker
        marker = Marker()
        marker.header.frame_id = "/base_footprint"
        marker.ns, marker.id = "ft_line", 0
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.25
        marker.points.append(Point(x=sin(t), y=cos(t), z=0.0))
        marker.points.append(Point(x=-sin(t), y=-cos(t), z=0.0))
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.8, 0.4, 0.4, 1.0
        marker_array.markers.append(marker)

        self.marker_publisher.publish(marker_array)

def main():
    rclpy.init()
    rclpy.spin(PathPlanningVisualiser())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

This code should publish a line and some text to the `/markers` topic which you can visualise in RVIZ.

{% figure src="/rviz add marker array.png" height="400px" /%}

Here is what the example should look like.

{% figure src="/sample RVIZ marker with text.png" height="300px" /%}

The marker types can be found here: [http://wiki.ros.org/rviz/DisplayTypes/Marker\#View-Oriented\_Text\_.28TEXT\_VIEW\_FACING.3D9.29\_.5B1.1.2B-.5D](http://wiki.ros.org/rviz/DisplayTypes/Marker#View-Oriented_Text_.28TEXT_VIEW_FACING.3D9.29_.5B1.1.2B-.5D).

By creating a different number of line markers and manipulating their positions such that they align with cones, you should be able to replicate the path planning visual present in this video:[Full MUR Autonomous Control Pipeline](https://www.youtube.com/watch?si=RwDqVK00hgsO23pK&v=sFjnP6knJ3M&feature=youtu.be).