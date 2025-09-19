The second objective of control is to send acceleration commands to the car. Sounds simple, right? Well firstly we need to ask ourselves 'what speed does the car need to reach?', 'how do we change the speed?', and 'how do we maintain our speed?'. 

If you send acceleration values to the car, it will eventually reach its target speed (which we call a 'setpoint'). However, without any feedback and adjustment, the car will keep accelerating without slowing down. 

## Error and P-Controller
So we need the car to know, whether it has reached its target or if it has overshot the setpoint. In this case, a error variable is needed. This is the difference between the current velocity of the car, and the velocity setpoint. Whatever the error value is at any given point in time, we will use this as our acceleration command. 

Makes sense. As the car approaches its setpoint, the acceleration will gradually reduce? Nope. If raw error values are used in the acceleration, the car will accelerate very quickly but may not slow down in time, and will overshoot, then it will jerk itself to stop, and undershoot. This is undesirable, because we want a smooth climb to the setpoint, with minimum overshoot. 

The error can be scaled down to, so that the car accelerates slower, but will not overshoot, or slam its brakes to slowdown. This scalar is called a **gain** .  
$$
a = k \cdot e
$$
The equation above is called a **P-controller**, where P stands for proportion. 

Now, you might be thinking, what value should 'k' be? The answer is, depends on the system. Because of physical properties like friction, inertia, mass 

## PI-Controller

Great, now we have control over how fast we want the car to accelerate. However, when the car reaches the setpoint, it may oscillate around it. This is because there is no mechanism that dampens the oscillation. It would be good if there was some way for the car to track its error, and gradually minimise its total error... 

This is where the I term (stands for integration), this term serves to dampen the car's acceleration, to help it maintain a consistent velocity. This term is added with the P term.
$$
a = k_{p}\, e \;+\; k_{i}\, \sum (e\, \Delta t)
$$
Let me explain the summation term; At every point in time, the error is multiplied by the elapsed time between the current and previous computation. This product is added to the accumulative error. 

During the acceleration phase, the accumulative error is positive, hence helping to accelerate the car faster, however, if the car overshoots, this term will become negative. This then dampens the acceleration, until the integral term becomes zero. 

## Practical Advise 01

In our experience, the I term needs to be kept small (<0.5), otherwise the car will overshoot and oscillate rapidly, and never reach the setpoint. Also, somethings the I term is not needed, because if the gain is small, the car will accelerate slowly and will stabilise at the setpoint. The I term is typically used for more aggressive acceleration, where the car needs to quickly slowdown after it overshoots the setpoint. 
