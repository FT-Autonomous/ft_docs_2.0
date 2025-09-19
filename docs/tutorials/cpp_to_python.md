### Note:

Its important to note that this is only a guide to migrate from Python to C++, not necessarily a mandatory read.
Many people prefer to work with Python for its dynamic types, but besides user readability C++ is the most optimal language for robotics as C++ is much closer to assembly language, both in terms of structure and performance.

## Reasons for wanting to migrate:

- C++ code runs miles faster than Python
- No dynamic types ( you know excatly what every variable should be storing )
- A very good way to understand how your code is actually working, and whether or not there are any mistakes in your current code.
- Avoiding bottle necks in performance for small but necessary ROS2 nodes and packages

## The Method:

Its important to note that I will be talking in the context of the pure_pursuit code for the control department as this is (to my knowledge) the first working implementation of 800+ lines of C++ code in the AI team.

### 1. CMakeLists.txt
Biggest apparant difference is the CMakeLists.txt. The CMakeLists.txt is a way for the build tool to compile C++ code and include all the necessary dependencies of the project. So for every import you see in pure_pursuit.cpp, you will likely see a corresponding reference in the CMakeLists.txt

In the CMake file you also need to define your project name and you can choose your compiler and include the necessary dependencies for your project:

```
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)                     # ROS2 syntax for C++ hence rcl"cpp"
find_package(std_msgs REQUIRED)                   # standard msgs for ROS2
find_package(ackermann_msgs REQUIRED)             # for ackermann
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)  
```

Its very important to be aware that these packages are directly to do with FT and the EUFS Sim and are required

```
find_package(ft_msgs REQUIRED)
find_package(eufs_msgs REQUIRED)
```

Then comes the adding of the executable in the form of ```add_executable(executable_name path/to/code)```

In this case: ```add_executable(pure_pursuit src/pure_pursuit.cpp)``` (Its important to know if you were ever wanting to change the launch file)

Make sure to have the relavent dependencies in this line!!! (One time I left out ```ft_msgs``` and the error logs were extremely misleading, they kept pointing me to a header file it couldn't find, cost me hours of time)
```ament_target_dependencies(pure_pursuit rclcpp std_msgs ackermann_msgs visualization_msgs geometry_msgs ft_msgs eufs_msgs tf2_geometry_msgs)```

As part of the ROS2 framework, ROS2 will actually generate the necessary header files for your messages ".hpp", the only catch is that you have to direct it where to go.

As you'll see in the code I'm currently using ```${CMAKE_CURRENT_SOURCE_DIR}``` which is by no means the best solution for dealing with paths so this part of the code is open to improvement.

And the rest is just other general stuff you'd see in any other CMakeLists.txt in any other ROS2 github project.

### 2. package.xml

**Note: as per the current configuration there are two packages, one for python, one for C++. To switch between the two, just rename one to package.xml and rename the other one to something other than package.xml e.g. packageOld.xml or packagePython.xml**

There are some slight changes relative to the original, more succinct python package. The C++ version has a few more dependecies but what is really important is the build_type for the export.

```
<export>
  <build_type>ament_cmake</build_type>
</export>
```

This is what defines that the code being built is using the CMakeLists.txt

If we instead use:

```
<export>
  <build_type>ament_python</build_type>
</export>
```

We will end up ignoring the CMakeLists.txt and just building the package to work with python code

### 3. pure_pursuit.cpp

The biggest difference that can be seen between the Python code and the C++ code is the type declerations. The types are quite long winded but overall are necessary.
For those who have programmed in C and C++ before, most of the conversions from python to C++ seem quite obvious but the less obvious conversions are generally the ones that deal with the ROS2 interface.

For example, take the Ackermann drive publisher:

First the publisher is defined in the private section of the class

Note: the cpp in rclcpp stands for C++

```rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;```

This is then initialized in the public section of the class:

```drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/cmd", 10);```

Note how all in all this is similar to setting any other variable:

type variableName = thing;

i.e.

type = rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr

variableName = drive_pub_

thing; = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/cmd", 10);

Also when it comes to types there is also ```auto``` but I've generally had more trouble with that

ChatGPT is actually quite good for helping figure out types but don't rely on it too heavily, if you really need it ask it one line at a time


#### Global variables

You'll notice a lot throughout the code the variables names often have an underscore following their name. This is common practice in ROS2 C++ to distinguish them from local variables.


#### Debugging

To debug code you can use the RCLCPP info logger:

```RCLCPP_INFO(this->get_logger(), "lookahead distance is: (%.4f)", ld_);```

There are also other types of loggers available such as WARN, but info generally does quite well


#### Messages

The two main messages types used for pure_pursuit are ft_msgs and eufs_msgs.

For ft_msgs the request message used is called "ControlRequest"

It consists of a variety of boolean and float64 variables which are useful in the pure_pursuit in deciding when the car needs to stop or follow the target velocity

Its important to know the underlying principle behind the messages as the syntax for messages is quite rough in C++ compared to Python

For example, this function is being called by a subscriber to ensure the values from the message are kept up to date

```
void PurePursuit::mission_control_cb(ft_msgs::msg::ControlRequest mission_control) {
    mission_control_ = mission_control; // I find it concerning that it can access that from here
}
```

## Conclusion

For this current pure pursuit code (as of 22/05/2025), testing done via the EUFS simulator and the FT Status tool has shown a 3X performance increase:

Python code runs at ~30 Hz consistantly

C++ code runs at ~90 Hz whilst driving and can max out at around 150 Hz whilst the car is at a standstill

Converting to C++ from Python is by no means an easy process, but if its as simple as converting 1 python file with less than <200 lines in a standalone package its in my opinion quite worth it considering the performance benefits and overall understanding of the code

> At the end of the day, the toughest problem FT faces, and will continue to face, is **knowledge transfer**, so converting some Python code to C++ is by no means any harm especially if it helps you understand the code better - Jake Casserly