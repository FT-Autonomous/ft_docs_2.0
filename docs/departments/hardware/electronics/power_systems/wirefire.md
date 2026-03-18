# Using High Power Electrical Systems

There are important safety measures to consider before using high power electronics

## what is 'high power'
in this context, we use high power to refer to any system that requires > 50W power (use formula P = V x I to cacluatepower)

## dangers
while in theory, we can know that our system will only need 7A, we run into some issues that occur when the circuit is set up in real life

1. No current protection - using the LiPo, if there is a sudden torque (current) spike due to inertia, say for instance, when starting a driving system from a stopped position, or initially powering on the Nvidia Jetson AGX as the fans spin up, the liPo will happily supply the higher required current until its fuse blows, or the required current drops to its steady state value.
1. Wire current limits - wires have rated currents that they can operate at. if they exceed this current, the wire will begin to melt (ask me how i know). this is bad, we dont want this. make sure you are using appropriate wires (16 AWG for 10A)

## Hazard reduction Measures
if you havent yet used a high powered electrocical system, or are unsure on anyhting safety related, reach out to a member of the hardware team for support (technical or emotional).

### overcurrent protection
Ensure fuses are placed startegically along the wiring loom to prevent high currents from destroying everything. fuses are cheap, Jetsons are not.

### arming sequence
after a particularly smoky test involving powering the velocity motor on rigby that resulted in the destruction of a mtor controller, it was decided to add in an arming sequence to any high power code: this consists of three stages:


- Boot: microcontroller goes through startup procedure
- Pre-arm: microcontroller ready, waits for a signal to say power system circuits are active
- armed: motor driver set to enable, allows power to flow from souce to motor.

currently, the arm signal must be set by hand, but we'll work in a circuit to send a signal automatically once system is armed.

thhis prevents any unintentional shorting that may damage the systems.


