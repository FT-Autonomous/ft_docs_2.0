
# Path Planning

The Path Planning department is responsible for developing the algorithms that allow the car to find a viable and ideally optimal "path" or trajectory around the track that the control submodule can execute reliably.
Path Planning receives as input cone (colour and coordinates on an Euclidean Plane) information from Perception and the current state of the car (current position and heading) and the map of the track from State Estimation .


### Goals

For 2024/2025, the Path Planning department aims to achieve the following goals:

- Complete the implementation of the existing Path Planning solution (Centerline generation). It currently seems to work well without any noise but with the addition of noise, similar to the real world, it exhibits erratic behaviour.
    - This is especially seen at turns
    - Arc-Centre Cone Virtualization  (ACCV) is a solution we came up with to deal with a large number of cones on one side of the track. The high level idea is as follows:
        The ACCV solution involves virtualising cones in the general area where
        they should be on the track when multiple cones are missed
        in close proximity. When the ”Cone Matcher” finds such
        a condition, the unmatched cones are used to roughly
        estimate the arc. The radii from these unmatched cones
        to the estimated center of the arc are calculated. Virtual
        cones are placed along these estimated radii as a guide
        for their positions. This approach to cone virtualisation
        is particularly useful when navigating curved sections of
        a track, when perception sensors' field of views might
        be skewed towards the ”outer” cones, leading to missed
        ”inner” cones.

- Once we have confidence in the centerline generation we hope to achieve "raceline-opitimsation" and incorporate some notion of "predictive" path and trajectory planning. (More on this in the "Path Planning Theory" section)

### Previous Implementations

#### 2022/2023

…

#### 2023/2024

In the 2023-2024 iteration of the solution we started from scratch and developed a new path planning solution called "FT Euclid", stripping away (but still taking a lot of inspiration from) the previous solution, "FT Euler" (2022-2023).
This path planning approach  works to generate centrepoints along the track and optimally interpolate these to generate a centreline and waypoints for the control team to follow. The centreline generation algorithm aims to robustly determine the track centreline from the detected cones, even in scenarios with missing or sparse cone data. This algorithm consists of three main components, the cone sorter, cone matcher and the core Path Planning Module (More on these in the "Path Planning Theory" section). We plan to incorporate the ACCV solution eventually into this system to imporove the robustness of the centerline generation.
…

### Resources

- FT Euclid Repository: <https://github.com/FT-Autonomous/ft_euclid> 