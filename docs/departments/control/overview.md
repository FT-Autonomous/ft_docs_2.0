
# Control Overview
Simply put, `pure_pursuit.py` and `pure_pursuit.cpp` are two **ROS (Robot Operating Software)** files which make up the two implementations of Formula Trinity's AI Control.

These **ROS nodes** listen for the car's pose (orientation and location of the car: `car_x`, `car_y`, `quaternion → car_yaw`) and a stream of race-line waypoints, pick a goal point a short distance in front of the car (the *lookahead* circle), compute a steering angle to reach that goal, and publish an **Ackermann drive command** (steer + accel).

---

## Subscribed Topics

- **/pose** — where the car is and how it is oriented.  
  Used to set `car_x`, `car_y`, `car_yaw`.

- **/raceline (PointArray)** — the sequence of `(x, y)` waypoints to follow.

- **Speed Source** — either  
  `/odometry_integration/car_state` for sim velocity, or  
  `/ros_can/wheel_speeds` for rear-wheel-based speed velocity.

- **Control overrides from the state machine** —  
  `/state_machine/safety_control` and `/state_machine/mission_control`  
  (stop/crawl/target velocity/accel).

---

## Published Topics

- `/cmd (AckermannDriveStamped)` — the actual steering angle and acceleration to the vehicle.  
- `/steering_angle (Pose)` for visualization and `/distance` (how far you've travelled).  
  The **C++ version** also publishes a marker for the goal point.

---

## Important Parameters

- **use_slam (bool)** — read pose from SLAM frames vs `base_footprint`; sets `frame_id`.  
- **nominal_speed**, **crawl_speed** *(m/s)* — baseline speeds.  
- **linear_speed_source** — `"sim"` or `"wheel_speeds"`.  
- **wheel_radius (m)** — used to convert wheel rpm to m/s.  
- **Lookahead distance `ld`** — computed as `L * coefficient` (wheelbase × multiplier).  
  In `pure_pursuit.py` it's a fixed `self.L * 1.2`.  
  In `pure_pursuit.cpp` it's parameterized via `ld_coefficient`.  
  This is the **#1 fine-tuning knob**.

---

## Glossary

- **Ackermann** — car-like steering (front-wheels turn), so we command steering angle and acceleration.  
- **Lookahead (`ld`)** — radius of circle centered at the car; we aim for a point where the raceline crosses the circle. Bigger `ld` looks further ahead.  
- **Alpha (α)** — the heading error between where the car points and where the goal point is.  
- **LAP / beta / gamma (C++)** — small add-ons to pure pursuit to account for curvature and how far off the path you are; they slightly shift the goal point for smoother tracking.

---

## Resources

Here's some helpful videos that describe what we typically deal with in Control.

Feedback Control: <https://www.youtube.com/watch?v=O-OqgFE9SD4&t=21s&pp=ygUQZmVlZGJhY2sgY29udHJvbA%3D%3D>

PID Introduction: <https://www.youtube.com/watch?v=UR0hOmjaHp0&pp=ygUQZmVlZGJhY2sgY29udHJvbA%3D%3D>

PID Demo: <https://youtu.be/fusr9eTceEo>

PID Steering: <https://youtu.be/4Y7zG48uHRo>

*Note, we don't use PID's for steering, but its a good illustration of how they work*

Pure Pursuit (long video) <https://youtu.be/x9s8J4ucgO0>

For more info on C++ code please see: 
- [Migration to C++](../../tutorials/cpp_to_python.md)