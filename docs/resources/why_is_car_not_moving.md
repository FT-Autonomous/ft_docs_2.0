# Why the F*ck is The Car Not Moving

You can do everything right all year but if at the competition you are unable to interface with the car and pass static and dynamic inspections you won't be able to do much. Making sure that real world factors are in order is as important if not more important than all other components in the stack.

<!-- TODO: Need to insert Panik image here
{% right %}
    {% image src="/panik.webp" height="200px" /%}
{% /right %} -->

Remember:  *Those who do not learn from the past are doomed to repeat it*.

### The Vehicle Emergency Brakes Immediately...

#### Improper ASR Handling

The ASR should not press the go signal until the vehicle has been in the `AS_READY` state for at least 5 seconds. This is indicated by the purple light blinking once on the ADS-DV ASSI.

##### Solution

Wait for the purple light before flicking the go toggle.

#### Internal Vehicle State Machine Issues

The vehicle should not receive any control messages before the vehicle enters the `AS_DRIVING` state. If the vehicle receives control messages two early, it will EBS immediately. Furthermore your control code should wait about four seconds after the vehicle enters the `AS_DRIVING` state to send control commands to wait for stability.

##### Solution

Add code to wait four seconds after entering the `AS_DRIVING` state before publishing to the `/state_machine/driving_flag` topic and/or publishing to the `/cmd` topic.

### The Vehicle Enters `AS_DRIVING` and Remains Stationary...

#### Insufficient Torque

In simulation almost all acceleration commands are able to get the vehicle to move even if at a slow speed. This is not the case in real life and there is a minimum torque value you need to actually get the car to move.

##### `ros_can` Sends Insufficient Torque By Design

We use the `ros_can` node from EUFS in order to to interface with the vehicle. The upstream version of `ros_can` is out of date in 2025 in the sense that it assumes that the vehicle uses four wheel drive. Therefore all torque requests to the vehicle are divided by two as the effective torque is doubled.

In the 2024 competition, the two four wheel drive ADS-DV vehicles (Alice and Bob) underwent mitosis and became four two-wheel drive ADS-DV vehicles (Alice, Bob, Carol and Dave). Dividing the torque by two essentially means that the vehicle will never get enough torque to move off the ground.

##### Solution

The simplest solution is to modify the `ros_can` c++ code to set the `torque_` value to `MAX_TORQUE_` (195 Nm) when the acceleration value provided is strictly greater than zero. This is guaranteed to work. Other teams use the maximum torque and then do wheel RPM control in order to get smoother control.

You should maintain your own fork of `ros_can` and keep it as a part of your stack. You can't simply consider it as a third party package that you can clone and have it work out of the box (unless it is patched by EUFS at some point in the future).

#### Insufficient Velocity

In simulation (at least in EUFS simulator), you can get the vehicle to go at very low speeds without any complications. In real life there are issues with this.

- To overcome rolling friction there is an initial velocity target that you have to surpass in order to get the vehicle to move.
- On the ADS-DV internal wheel RPM feedback can be non-zero even if the vehicle is completely stationary.

##### Solution

Make sure that you set your nominal speed to at least 1.5m/s in real life.