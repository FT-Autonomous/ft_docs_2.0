# Hardware

Hardware is the newest department to FT Autonomouts (as of Septermber 2025) and serves to build the autonomous test rig "Rigby" and to improve the sensor mount.

## Goals
### General
- Create and maintain a working test rig 'Rigby'.
- Simulate a DDT car using Rigby.
- Design interchangeable sensor mount for DDT car and Rigby.
- Support rest of FT Autonomous with Hardware assistance.
### Current year (2025/2026)
#### Electronics
- Implement potentiometer based control loop for steering system.
- Finalise and implement communication protocols for Jetson to Jetson, and Jetson to Arduino communtications.
- Integrate independant power supply for drive and steering motors onto rigby.
#### Mechanical
- mount poteentiometer to Rigby for control of steering.
- Add a more permanant weatherproofing solution to the sensor mount.
- Add mounting plate to Rigby, to give more space for onboard electronics and morot power supply.

## GitHub Overview
FT-Hardware  
├─&nbsp;CAD  
│&nbsp;&nbsp;├─&nbsp;Circuit&nbsp;Diagrams  
│&nbsp;&nbsp;├─&nbsp;Comp&nbsp;Resources  
│&nbsp;&nbsp;├─&nbsp;Mount  
│&nbsp;&nbsp;│&nbsp;&nbsp;├─&nbsp;[Design&nbsp;processes&nbsp;for&nbsp;printed&nbsp;mount&nbsp;components]  
│&nbsp;&nbsp;│&nbsp;&nbsp;├─&nbsp;[Previous&nbsp;years'&nbsp;mounts]  
│&nbsp;&nbsp;│&nbsp;&nbsp;└─&nbsp;[Current&nbsp;year's&nbsp;mount]  
│&nbsp;&nbsp;└─&nbsp;Rigby  
│&nbsp;&nbsp;&nbsp;&nbsp;├─&nbsp;3D&nbsp;Printed&nbsp;Parts&nbsp;(reproducible)  
│&nbsp;&nbsp;&nbsp;&nbsp;│&nbsp;&nbsp;└─&nbsp;[SolidWorks&nbsp;files]  
│&nbsp;&nbsp;&nbsp;&nbsp;├─&nbsp;Bought&nbsp;Parts&nbsp;(to&nbsp;be&nbsp;replaced)  
│&nbsp;&nbsp;&nbsp;&nbsp;└─&nbsp;Printable&nbsp;Parts  
│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;├─&nbsp;Bambu&nbsp;A1  
│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│&nbsp;&nbsp;├─&nbsp;3MF  
│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│&nbsp;&nbsp;└─&nbsp;GCODE  
│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;├─&nbsp;Prusa  
│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│&nbsp;&nbsp;├─&nbsp;3MF  
│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│&nbsp;&nbsp;└─&nbsp;GCODE  
│&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;└─&nbsp;STL  
│  
└─&nbsp;Code  
&nbsp;&nbsp;└─&nbsp;[WIP]  

## Previous Implementations

### 2022/2023

Rigby was made summer 2023, so no Hardware work was done in this time period

### 2023/2024

- First pass at a steering improvement on rigby, changed motor
- First pass at electronics systems, steering angle feedback

### 2024/2025
- Improve steering solution to remove toe out geometry
- Redo electronics wiring
- Implement feedback sensors and PID control, and modular control systems (Arduinos and Jetson)


## Resources

… (list of links here)
