# Control Interfaces

Rigby has two normal input routes. Both end at the same drive and steering controllers, but they accept different messages. Select one in `rigby.toml`; do not run both runtimes against the serial ports.

## PEAK-CAN / DDT Mimic

The default `peak_can` route leaves the Jetson's normal stack and `ros_can` on the AI side. Rigby implements the vehicle/VCU side using Linux SocketCAN, normally `can0`, classic 11-bit CAN at **500 kbit/s**.

The implementation is pinned to FS-AI API revision `33bcb61d2519ee1ac4935396d9bf465c6ebc81c1`. Its exact byte layouts are in [FT-Rigby's protocol contract](https://github.com/FT-Autonomous/FT-Rigby/blob/main/docs/DDT_PROTOCOL.md). Do not substitute a newer DBC without reviewing the codec and tests.

| AI command ID | Purpose | Required DLC |
| --- | --- | --- |
| `0x510` | Status, handshake, mission, direction and stop request | 8 |
| `0x511` | Front drive torque and RPM ceiling | 4 |
| `0x512` | Rear drive torque and RPM ceiling | 4 |
| `0x513` | Steering angle | 2 |
| `0x514` | Brake request | 2 |

All five are required, including frames whose values are zero. Wrong-length frames do not refresh their timers. The implemented command and handshake deadlines are 100 ms; these are Rigby's test-interface settings, not a declaration of competition-rule compliance.

Feedback includes status, accepted requests, measured steering/speed, wheel counts and diagnostic/logger frames. Most feedback runs at 100 Hz, with logger status at 10 Hz. Emulated torque capability and DDT wheel geometry are protocol values, **not Rigby's measured torque rating or physical wheel dimensions**. Hydraulic brake feedback remains zero without pressure sensors.

### From OFF to DRIVING

The implemented sequence requires fresh command frames, a completed handshake, valid actuator telemetry and the required local ASSI reports. READY lasts at least five seconds. Entry to DRIVING also checks neutral/zero requests and steering near centre.

In the normal ASSI path, the local GO signal is an eligible-window `SIGUSR1`, followed by an ASSI `D` report. An early signal is not queued for later. This description is not a launch command: the [open audit findings](../testing.md#controls-audit-open-findings) include stop-event and ASSI-state defects.

Communication faults are intended to enter a latched emergency state. Software state changes do not give Rigby the competition car's physical emergency-braking hardware.

### Drive Translation

The Jetson requests torque and an RPM ceiling, not a linear speed. The mimic takes the lower front/rear RPM ceiling, uses the pinned DDT conversion, and applies Rigby's local speed cap. Zero torque disables drive output rather than maintaining an old speed request; it allows coasting, not hydraulic braking.

Reverse remains disabled by default. Simultaneous drive/brake requests and other implausible combinations are handled by the emulated state machine. This is a software contract to test, not proof of stopping distance.

## ROS Ackermann

The optional `ros_ackermann` route accepts `ackermann_msgs/AckermannDriveStamped` on **`/cmd`** over the ROS network. It does not require PEAK-CAN. The Jetson and host need a compatible ROS environment/domain and actual network connectivity.

By default, the bridge integrates the message's acceleration into a bounded speed target (`use_speed_field = false`). Setting `use_speed_field = true` selects the speed field instead. Steering arrives in radians and is converted to controller degrees. Check the selected launch parameters before publishing: the right topic is not enough if the publisher and bridge disagree about which field drives the speed target.

Command receipt must remain fresh within the configured 0.5 s window. The audit found a ROS-clock freshness problem and an unresolved active-steering-centre behaviour on authority loss. Do not describe those intended checks as fully validated fail-safe behaviour.

## Steering Sign and Feedback

The external convention is **positive left**. Rigby's actuator convention is the opposite. With `invert_steering = true`, a +7.5 degree external request becomes -7.5 degrees at the steering controller, and measured feedback is converted back.

Keep the sensor calibration separate from the operational limit. The host may permit only +/-7.5 degrees while the potentiometer calibration spans +/-17.5 degrees. Actual overshoot must remain visible; clamping feedback to the request limit would hide it.

## Physical Integration

Virtual CAN, simulated serial endpoints and localhost ROS tests are useful, but do not check connectors, termination, wiring, motor direction or the real Jetson link. Use the [PEAK setup reference](../../../resources/peak_pcan_usb_socketcan.md) and FT-Rigby's [acceptance guide](https://github.com/FT-Autonomous/FT-Rigby/blob/main/docs/INTEGRATION_ACCEPTANCE.md), then record the physical test separately.
