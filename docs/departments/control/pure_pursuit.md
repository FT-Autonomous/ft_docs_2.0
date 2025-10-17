# Pure Pursuit — Formula Trinity AI Control

**Pure Pursuit** is a geometric path-tracking algorithm used in autonomous vehicles to compute the steering angle needed to follow a predefined path of waypoints.  

It belongs to the **Control Layer** of the Formula Trinity stack, taking in where the car is (pose) and where it should go (raceline), and outputting steering commands.

By constantly looking ahead a distance \( l_d \) (the *lookahead distance*) on the raceline, the algorithm identifies a **goal point** — where the raceline intersects a circle of radius \( l_d \) centered on the car.  
By steering toward this point, the car naturally follows the path in smooth arcs.

---

## The Pure Pursuit Loop

### 1. Read State

From `/pose`:  
Extract `car_x`, `car_y`, and `car_yaw`.  
`car_yaw` is computed from the quaternion using a quaternion-to-angle helper function.

---

### 2. Pick Waypoints Around the Lookahead Circle

1. Compute the distance from the car to each race-line waypoint.  
2. Find:
   - The **nearest waypoint** (inside the lookahead circle), and  
   - The **next waypoint** just *outside* the lookahead radius \( l_d \).  
3. These two points define a line that crosses your lookahead circle.

---

### 3. Find Circle–Line Intersections

- The line through those two waypoints is intersected with the lookahead circle centered on the car.
- Two intersection points result — pick the one **closer to the outside waypoint**.  
  This point becomes the **goal point**.

---

### 4. Turn Toward the Goal

Compute the heading error \( \alpha \):

\[
\alpha = \text{angle between the car's heading and the vector from car → goal.}
\]

The **Pure Pursuit steering formula**:

\[
\delta = \tan^{-1}\!\left(\frac{2L\sin(\alpha)}{l_d}\right)
\]

where:
- \( \delta \): steering angle  
- \( L \): wheelbase (distance between front and rear axles)  
- \( \alpha \): heading error  
- \( l_d \): lookahead distance

---

### 5. (C++ Only) Grip-Aware Helpers — *Beta, Gamma, Tau, and LAP*

The C++ implementation adds an extra layer:
- Estimates path curvature.
- Mixes parameters:
  - **Gamma (γ)** — how far off the path the car is.
  - **Beta (β)** — curvature gap.
- Nudges the goal point to a **Look-Ahead Point (LAP)** and recomputes steering toward it.

This results in smoother path-following — think *“Pure Pursuit + correction for curvature/offset.”*

---

### 6. Decide Acceleration

- If **safety** or **mission control** signals stop/crawl/target velocity/accel — those override.
- Otherwise:
  - Compute a **target velocity** (rule-based in C++, nominal in Python).
  - Use a **P/PI/PID** control to move the actual velocity toward it.
  - Clamp acceleration by `max_accel`.

(Python uses proportional-only control; C++ includes a full PID scaffold.)

---

### 7. Publish Results

- Send `/cmd` (AckermannDriveStamped) with steering angle and acceleration.  
- Update `/distance` by integrating velocity over time.  
- Publish visualization topics for steering and goal point markers.

---

## Mathematical Summary

Let:

| Symbol | Meaning |
|---------|----------|
| \( (x, y) \) | Current car position |
| \( \theta \) | Current car heading (yaw) |
| \( L \) | Wheelbase (distance between axles) |
| \( l_d \) | Lookahead distance |
| \( \alpha \) | Angle between car heading and line to goal point |
| \( \delta \) | Steering angle |

Then the steering angle is given by:

\[
\delta = \tan^{-1}\!\left(\frac{2L\sin(\alpha)}{l_d}\right)
\]

---

## Step-by-Step Mathematical Process

### 1. Find Intersection

Compute the distance between the car and all nearby waypoints:

\[
d^2 = (w_x - x_{\text{car}})^2 + (w_y - y_{\text{car}})^2
\]

where \( \vec{w} = (w_x, w_y) \) are path waypoints.

Then:

\[
d_c = \sqrt{d^2} - l_d
\]

Find the index of the smallest value in \( d_c \):

\[
\text{index} = \operatorname*{argmin}(d_c)
\]

The waypoint with the smallest \( d_c \) is closest to the lookahead circle circumference.

---

### 2. Find the Angle \( \alpha \)

\[
\alpha = \tan^{-1}\!\left(\frac{y_{\text{goal}} - y_{\text{car}}}{x_{\text{goal}} - x_{\text{car}}}\right)
\]

If \( \alpha < 0 \):

\[
\alpha = \pi - \alpha
\]

Adjust for the car’s yaw:

\[
\alpha = \alpha - \gamma
\]

Then compute steering:

\[
\theta = \tan^{-1}\!\left(\frac{2L\sin(\alpha)}{l_d}\right)
\]

---

## Visualization

<figure markdown>
  ![Pure Pursuit Concept](media/image1.png){ width=600 loading=lazy }
  <figcaption>Figure 1: The car follows the raceline by targeting a point on the lookahead circle.</figcaption>
</figure>

---

## Glossary

| Term | Description |
|------|--------------|
| **Ackermann** | Car-like steering (front wheels turn), so we command steering angle and acceleration. |
| **Lookahead (ld)** | Circle radius centered on the car; the raceline’s intersection defines the goal point. Larger \( l_d \) = smoother turns. |
| **Alpha (α)** | Heading error between where the car points and where the goal point is. |
| **LAP / Beta / Gamma (C++)** | Add-ons to account for curvature and path offset, improving smoothness. |

---

## References

- [Purdue SIGBots Wiki — Basic Pure Pursuit](https://wiki.purduesigbots.com/software/control-algorithms/basic-pure-pursuit) — an excellent resource on autonomous vehicle control.

---

*Pure Pursuit is an automatic steering control system — the car continuously turns to follow a smooth trajectory along the given path.*
