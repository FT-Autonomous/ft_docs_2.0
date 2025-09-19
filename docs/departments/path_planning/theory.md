
# Path Planning Theory
The goal of the path planning subsystem (which we have
nicknamed ”FT Euclid”) is to produce a path which will be executed by the control module. They take as inputs the cone
information (position and colour) from the perception team, the
map of the track and the current pose of the vehicle computed
by the state estimation team.
The path planning approach currently works to generate
centrepoints along the track and optimally interpolate these to
generate a centreline and waypoints for the control team to
follow. The centreline generation algorithm aims to robustly
determine the track centreline from the detected cones, even
in scenarios with missing or sparse cone data. This algorithm
consists of:

- Cone Sorting The visible cones are sorted relative to the car’s
position and direction of movement using a ”Cone-Sorter”
submodule.

- Cone Matching The ”Cone-Matcher” submodule attempts to
match cones on one side of the track to their counterparts
on the other side, i.e. make pairs of cones. If no match is
found for a particular cone, it attempts to place a virtual
cone. This allows for robust centreline generation, that
results in a centreline that is very closely representative of
the actual centre of the track, even with missed cones. If
a series of cones (3 or more) is found not being detected
very closely, which is often the case for the inner side
of the track on tight bends, we employ Arc-Centre Cone
Virtualization (ACCV) described below.

- Centrepoint Calculation The midpoints of pairs from the cone
matching form the basis of our centreline path. These
midpoints are interpolated using a suitable clothoid spline
fit along with parametric curves. This gives control over
the final path that, i.e. adjust and optimise the generated
based on objective parameters and constraints, ensuring
continuity and smoothness.

- Arc-Centre Cone Virtualization (ACCV). The ACCV solution involves virtualising cones in the general area where
they should be on the track when multiple cones are missed
in close proximity. When the ”Cone Matcher” finds such
a condition, the unmatched cones are used to roughly
estimate the arc. The radii from these unmatched cones
to the estimated center of the arc are calculated. Virtual
cones are placed along these estimated radii as a guide
for their positions. This approach to cone virtualisation
is particularly uselful when navigating curved sections of
a track, when perception sensors’ field of views might
be skewed towards the ”outer” cones, leading to missed
”inner” cones.

- Testing Path Planning is primarily visual problem and console
outputs don’t provide significant material insights into the
evaluation and effectiveness of the solution. Additionally
python’s generic plotting libraries are not as customisable
as desired, as a solution to this, on top of the routine unit
and integration testing, we use RViz to plot and visualise
our Path Planning solution. This lets us stress test in a
variety of conditions, letting us pinpoint vulnerabilities and
adapt the solution to these.

The waypoints generated from the interpolation of the mid-
points are published to a ROS topic for control to use. Even-
tually, the aim for path planning is to be able to produce an
optimised ”raceline” to follow over a time horizon in the laps
subsequent to the first lap; taking in a global map and the
”centreline” as inputs.

### Arc Centre Cone Virtualisation (ACCV) 

*ACCV is a technique used to estimate the position of missing perception cones.*

#### Requirements for ACCV 

- Minimum of three cones per side 
- Both sides of cones must be non-collinear (threshold defined in utils)
- Number of detected missing cones is below 80% 

#### Our approach 

The majority of this functionality is implemented in Utils/track.py and Utils/accv.py with some utility functions implemented in Utils/mathUtils.py. 

- Calculated average distance between cones for both sides
- Computed threshold using 25th and 75th percentile of average distances 
- Using average distance threshold, insert "PLACEHOLDER" cone to indicate a missing cone.
- If the proportion of detected cones is sufficinet (above 80%), bypass ACCV. 
- Using the check_curviness() function in mathUtils, collinearity is checked. If below a threshold, a simple linear mapping is used. 
- Using the side with more detected cones, the missing cones are virtualised using ACCV.
- The centre and radius of the circle are calculated using a minimum of three cones using the _best_fit_circle() function. 
- The ACCV function itself maps each detected cone to the undetected side using the angle it creates with the center. If detected cones are on the outer side, virtual cones are placed on the inner side (radius - path_width). Otherwise, virtual cones are placed on the outer side (radius + path_width).
- These virtualised cones along with the original detected cones are then used in the centrepoint calculations. 