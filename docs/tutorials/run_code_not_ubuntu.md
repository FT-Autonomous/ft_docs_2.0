---
title: Running our Code on Linux Distributions (That Are Not Ubuntu)
---

## Running our Code on Linux Distributions (That Are Not Ubuntu)

Some people run Linux distributions such as Arch Linux. These people need to be able to use our codebase as well.

### Compiling from Source

You can attempt to compile ROS2 from source. There [instructions on the official ROS2 wiki](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html). There are also recipies on the AUR for compiling from source on Arch Linux based distributions.

Compiling from source is not reccomended:

- ROS2 developers do not really have the time to guarantee that things work on distros that are not Ubuntu.
- Compiling from source takes a long amount of time.
- The time you spend trying to get protobuf versions to match, while at the same time keeping libraries working with the rest of your system, and trying to resolve undefined referenced issues due to mismatched shared libraries is not really worth it (unless you're using something like NixOS that makes this easier).

### Virtual Machines

This is probably the only way to do things on-device if you're running MacOS.

### Docker

First install [docker](https://docs.docker.com/engine/install/) or [podman](https://podman.io/docs/installation) first.

You want to navigate the [ft-ubuntu-bootstrap](https://github.com/FT-Autonomous/ft-ubuntu-bootstrap) repository and build the docker image described in the Dockerfile there. This essentially involves the following steps:

```
mkdir ~/ft
cd ~/ft
git clone https://github.com/FT-Autonomous/ft-ubuntu-bootstrap
cd ft-ubuntu-bootstrap
docker build -t ft/base -f base.Dockerfile .
```

Once the image has built, you will want to use the image as follows:

```
cd overlay/
python3 enter-overlay.py humble
```

The concept of the overlay is described in the [README.md of `ft-ubuntu-bootstrap`](https://github.com/FT-Autonomous/ft-ubuntu-bootstrap/blob/master/README.md).

### Using the FTA PC

The FTA PC is a remote computer that all FT members are given access to. If you are unable to run ROS2 code on-device, ask a department lead to set you up with the FTA PC.