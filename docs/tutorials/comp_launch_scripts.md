---
title: "Competition Launch Scripts: Configuration, Setup and Running them"
---

## Competition Launch Scripts: Configuration, Setup and Running them

This guide assumes that you are running on some distribution of Ubuntu, preferrably Ubuntu 22.04. It should work in either WSL or a native Linux install. 
And have the FT-FSAI2x stack, EUFS simulator and gazebo installed. There is a guide on how to do that [here](../tutorials/Getting_started.md).

### What are the Competition Launch Scripts?
These can be found in [ft_system/src/launch](https://github.com/FT-Autonomous/ft_system/tree/0534ff4df05545cf648c2c7c08044ef239071513/src/launch)
We run this script on the car, but can be used in simulation (instead of ```system_sim.launch.py``` )as well.

- autocross.launch.py
- acceleration.launch.simulation.py

### How to run the Competition Launch Scripts in simulation

#### Setting up the environment

Before running, check that you have the ```FT_CATKIN_HOME``` environment variable set (this is needed for perception and some ROS1 Packages)

You can use this to list your environment variables and what they are set to:
```bash
env
``` 
If it is not set, you can set it using the following command to set it to ```~/catkin_ws``` or your FT Workspace (this should be set up on the FTA pc):

```bash
export FT_CATKIN_HOME=$HOME/catkin_ws    #replace catkin_ws with your FT workspace if it does not exist

```
or if you want it to be permanently set :
```bash
echo 'export FT_CATKIN_HOME=$HOME/catkin_ws' >> ~/.bashrc  #replace catkin_ws with your FT workspace if it does not exist
source ~/.bashrc
```

#### Launching

Then, you can run the launch script using the following commands in your FT Workspace:

Make sure you source on every terminal/terminal tab you use:
```bash
. install/setup.sh  #(or any alias you might have set up for this, most people have it set to .w or .r)
```

Compile all of our code along with the simulator code. You can do this with the following command.

```bash
colcon build --symlink-install
```

To launch EUFS Simulator:
```bash
ros2 launch eufs_launcher eufs_launcher.launch.py
```

in another terminal tab, To run the FT Stack launch script:
```bash
ros2 launch ft_system autocross.launch.py simulation:=yes
```