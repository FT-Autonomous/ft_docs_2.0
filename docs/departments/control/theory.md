# Control Theory

- This section covers the theoretical aspects of the Control department.
- Detailed explanations of algorithms and methodologies used.
- Research papers and references.

## ADS Theory

### First attempt at design solution:
* Clutch will be controlled by a hardware driven PID
* Accelerator pedal will also be driven by a PID, but we can vary the setpoints based on input when the clutch is up

* To get the car moving, we will require a combination of both clutch and accelerator pedal:
* Its likely that we will implement stages such that the start off procudure is entirely code driven.

* We plan on using an xbox controller or something similar to make proper tests for the ADS car

### Look at first commit to ads repo to see layout of ADS algorithm

*  Most important paramaters:
*  RPM
*  ...