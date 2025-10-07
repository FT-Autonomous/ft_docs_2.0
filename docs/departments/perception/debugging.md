# Debugging Code

If there are less cones shown in "rviz2" than what you expected, make sure to check the allowed confidence level (that filters out cones predicted from yolo) in the file so that its not too low that it takes any yellow object as a cone or not to high that we have less number of cones predicted from yolo