# Jetson Nano
The NVIDIA Jetson Nano (Jetson), is a GPU accelerated edge computing device intended for GPU and computationally heavy tasks.

## Hardwares Use Case
The Jetson is used by the hardware team as a centralised 'brain' of rigby's onboard control systems. It recieves data from the software stack on the main Jetson AGX, realted to desired velocity and steering angle, and forwards the relevant signals on to the corresponding microcontroller via a serial port. Using the same serial port, the microcontrollers send the current control data back to the Jetson, where it is then sent back to the main software stack for the control script.

## How does it recieve/send data to/from Control?
The Jetson recieves and sends the data to/from the main Jetson AGX using ROS2 nodes. As the main software stack is focused around ROS2 nodes, its easier for hardware to create their own nodes to interact with the system than any other workaround solution. this is currently a work in progress and has not yet been implemented.

## How does it recieve/send data to/from the microcontrollers
The Jetson recieves and sends data to and from microcontrollers using serial/UART communication protocols. using the Pyserial package, we can use python scripts to send data throgh a serial port to the microcontroller, where it can read it from its serial buffer. The microntroller then sends some other data back by writing to its serial port, where Pyserial can then pick it up and save it.

(in future, this could be done through WI-FI if we hate ourselves enough, but as of right now, it is not reccommended)

## Why a NVIDIA Jetson Nano?
Its clear to see that a Jetson is blatantly overkill for hardwares uses, but there are a few main reasons why one is used for this purpose:
- The Jetson has four USB ports, meaning it can power and communicate with all the required microcontrollers for control and UI.
- The Jetson is powerful enough computationally to support the ROS2 nodes.
- We had one going without use, so we didnt have to buy an alternative

## What about ADS
for the ADS car, hardware has adapted the use case of the Jetson to functionally do the same role, but with more controllers (including one using a modbus/TCP protocol). As a new purchase is required, it will be replaced with a Raspberry Pi 5 alternative.
