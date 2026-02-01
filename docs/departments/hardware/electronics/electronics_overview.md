# Electronics

The Electronics side of Hardware looks over all Electrical and Electronic aspects of Rigby and the Sensor Mount, these

## Rigby
### Computational Devices
There are three main computational devices on Rigby:
	- NVIDIA Jetson Nano
	- Arduino Uno R4 WIFI
	- Arduino Nano 33 IOT
#### NVIDIA Jetson Nano
The NVIDIA Jetson Nano (Jetson), is a GPU accelerated edge computing device intended for GPU and computationally heavy tasks. In hardware, it is used as a Central Computational Device, managing the signals sent to and from the arduinos which directly interface with the motors via serial bridges (USB) at a high level, and Recieve signals from the rest of the stack on the NVIDIA Jetson AGX

#### Arduino Uno R4 WIFI
The Arduino Uno R4 wifi (R4), is a microcontroller device that we use in Hardware to interact between the Jetson and the motors. There is one each for the steering and driving motors. Each R4 contains code to interface with the mtor drive by sending a Pulse Width Modulated signal. the R4s operate on a 5V logic level, meaining signals it outputs will be at 5V.

#### Arduino Nano 33 IOT
The Arduino Nano 33 IOT (33 IOT) microcontroller device is used in Hardware to implement the AMS/ASSI functionaltiy onto the system. It is desgined to be mounted onto a breadboard or perfboard, and operates at a 3.3V logic level. It connects to several LEDs and buttons to allow selection of mission, and to indicate current state of the car.

IMPORTANT: DO NOT SEND 5V SIGNALS TO THE 33 IOT, IT WILL KILL THE DEVICE.

### Wiring
Rigby

## Mount
