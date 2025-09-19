# Getting Started Guide

This guide assumes that you are running on some distribution of Ubuntu, preferrably Ubuntu 22.04. It should work in either WSL or a native Linux install. We're working on making an article about how to run the stack on distros such as Arch Linux [here](../tutorials/run_code_not_ubuntu.md).

### Prerequisites

Most of our setup scripts are stored in git. Start off by cloning the repository containing our setup scripts.

```
mkdir ~/ft
cd ~/ft
git clone https://github.com/FT-Autonomous/ft-ubuntu-bootstrap
```

### Installing ROS2 and Gazebo

ROS2 stands for "Robot Operating System 2". It's a framework that makes it easy to architect robotics and automotive systems. Gazebo is a physics simulator that we use to test our code. The standard installation guides are as follows:

- [ROS2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [Gazebo Installation Guide](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)

The main reason these links are provided are for if you want to go through the installation yourself piece by piece. As a shortcut, you can avail of our pre made setup scripts and simply run the following commands, assuming you have downloaded the `ft-ubuntu-bootstrap` repository previously.

```
sudo bash ft-ubuntu-bootstrap/get-ros
sudo bash ft-ubuntu-bootstrap/get-gazebo
```

Make sure that you have ROS2 sourced in your terminal environment. If it is not already present, add the following line to your `~/.bashrc` file.

```bash
. /opt/ros/humble/setup.bash
```

If this line was not already present, after making the change, you will need to start a new terminal. In the new terminal, you can check whether or not ROS2 has been sourced using  the following command:

```bash
ros2
```

If everything worked out fine, this should print out all of the subcommands available under the main ros2 command.

### Cloning our Repository

Our main repository, `FT-FSAI-23` is private. In order to clone it, you will need to set up git credentials. Please follow this guide here on how to do that:

- [Our Guide on how to Setup Git Credentials](../resources/git.md)

Once you have set up git your credentials, you can then clone our code and the code for the simulator:

```bash
cd ~/ft
git clone https://github.com/FT-Autonomous/ft-ubuntu-bootstrap
bash ft-ubuntu-bootstrap/get-eufs
gh repo clone FT-Autonomous/FT-FSAI-23 -- --recurse-submodules
```

If you are not using the GIT CLI, you should follow [this guide from GitHub](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) to add an SSH key to your account. Once that key has been added, you can run the following command.

```bash
git clone --recurse-submodules git@github.com:FT-Autonomous/FT-FSAI-23
```

If you get an error complaining that the repository doesn’t exist, you may not be a member of the GitHub organisation, so ask somebody to add you.

If you get an error about missing submodules(EX: slam_tools) ensure they are properly installed by running the following commands.

```bash
cd FT-FSAI-23
git submodule init
git submodule update
```

### Getting Set Up for ROS Development

The next step is to compile all of our code along with the simulator code. You can do this with the following command.

```bash
cd ~/ft
colcon build --symlink-install
```

**Note**: There’s currently a bug in the repository which causes ft\_fsd to fail to compile the first time around. If you see an ft\_fsd error, run things again. This will be fixed soon.

Colcon is one of the main build tools for ROS2. The `colcon build` command is the only one that you will ever use. After compiling your code in the `~/ft` folder, you need to import all of  the compiled packages into your terminal using the following command:

```bash
cd ~/ft
. install/setup.sh
```

The dot in bash is similar to the import statement in python. Once you have used that command, you should try to run the simulator. You will need to source the contents of the \`\~/ft\` workspace every time you want to run the simulator.

Pro Tip: You can shorten this by adding the following command to your `~/.bashrc`. It defines a `.w` command that navigates to your workspace and imports all dependencies in one command.

```bash
alias .w='cd ~/ft ; . install/setup.bash'
```

Pro Tip: If you get an EUFS\_MASTER error, just close the terminal, open it again and ‘cd’ back into ft. That should solve the error.

```bash
ros2 launch eufs_launcher eufs_launcher.launch.py
```

## Running the System

Once you have EUFS running, you can run the whole stack using the following command:

```
. install/setup.bash
ros2 launch ft_system system_sim.launch.py
```