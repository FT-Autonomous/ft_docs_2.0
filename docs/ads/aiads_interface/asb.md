# Autonomous System Break

The ASB serves to allow the autonomous system to slow itself down, however the supervisor for this system is very mission critical and such it also has the option to trigger the ShutDown Circuit and in turn the Emergency System Break. The supervisor should monitor the other electronic systems to ensure they do not stall (if they do: SDC), and is itself monitored by a hardware watchdog that requires a heartbeat or else it will trigger the SDC.

## What signals does it monitor and what does it care about them

It monitors the CANbus to ensure that no system has stalled. Every system should update the values associated with it every X amount of time. If they fail to do so, the supervisor should trigger the SDC. 100ms is a reasonable time for this if CANbus is at 50Hz (FSG Rules define 500ms as the longest digital signal timeout permissable).

- Pi/Jetson sends heartbeat through the PCAN link
- ESPs sensor data changing should constitute a heartbeat
- Test to ensure it is able to detect a broken wire, a short to ground, an analogue sensor short to supply, an implausible or out-of-range sensor value, and corrupt, missing or delayed digital messages.