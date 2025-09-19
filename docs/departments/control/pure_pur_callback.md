*In this document, the functions of the Pure Pursuit node are given and explained in detail. On github, this is the "korean_pure_pursuit" branch, that was deployed in the 2024 competition*
# Class

The majority of the code is inside the Pure Pursuit class, which is called in the mains function. This class acts as the node which ROS communicates to. 
# Functions 

There's a lot of functions in this script alone, and it can be hard to navigate. Here, I'll cover the most important ones first, then work backwards from there. 

# self.cb
This is the most important function, as this is where all our computation takes place, and where most of the other functions are called. You can image that is our 'mains()' function, but inside a class structure. 
### Step 1 : Find Goal Point 

The first task of self.cb is to determine the car's state (so, where are we?), this takes in the current x y coordinates (called 'pose'), and direction the car is facing (called 'yaw'). The function *quaternion_to_angle(data.pose.orientation)* converts the yaw from a quaternion to an angle, which we can compute more easily with. 

	if self.get_parameter("use_slam").get_parameter_value().bool_value:
            self.car_x = data.pose.position.x
            self.car_y = data.pose.position.y
            
            self.car_yaw = quaternion_to_angle(data.pose.orientation)
            self.frame_id = "odom"
        else:
            self.frame_id = "base_footprint"


 If there are waypoints available, the code will try to find the nearest waypoints inside and outside the look ahead circle. It calls **self.calc_nearest_index()** for this task. Next it computes the intercepting lines using **self.find_lookahead_intercepts()**. 

	if self.waypoints_x and self.waypoints_y:
            n_wp_inside_x, n_wp_inside_y, n_wp_outside_x, n_wp_outside_y = self.calc_nearest_index(self.waypoints_x, self.waypoints_y)
            intercept_1_x, intercept_1_y, intercept_2_x, intercept_2_y = self.find_lookahead_intercepts(n_wp_inside_x, n_wp_inside_y, 
                                n_wp_outside_x, n_wp_outside_y)

    goal_x, goal_y = self.find_goal_point(intercept_1_x, intercept_1_y, 
                    intercept_2_x, intercept_2_y, n_wp_outside_x, n_wp_outside_y)

Once it has the nearest waypoints, and the intercepts, next we compute the goal point coordinates. The function **self.find_goal_point()** handles this step. 

### Step 2 : Calculate Steering Angle 

	alpha = self.find_alpha(goal_x, goal_y)
          
    angle = self.calculate_steering_angle(alpha)

However, with the latest steering control algorithm, there are additional steps needed. We need to calculate the Look Ahead Point. To do so, we need to calculate the parameters for the translation equations, called **gamma, beta and tau**. These have their own functions for calculation.

	gamma = self.find_gamma(np.array([self.car_x, self.car_y]))
            beta = self.find_beta([n_wp_inside_x, n_wp_inside_y], angle)
            tau = (1 - gamma) * beta

Next, the translation equations are computed, this moves the goal point along the x and y direction, depending on the value of tau. **length_gc** is the distance between the car and goal point, and its computed using a *line equation* (remember from secondary school?). Finally, the look ahead point ("lap") is calculated. 

	length_gc = math.sqrt( (n_wp_inside_y - self.car_y)**2 +
                                  (n_wp_inside_x - self.car_x)**2 )
            
    lap = [goal_x + (tau * length_gc) * math.cos(angle),
                   goal_y + (tau * length_gc) * math.sin(angle)]

Now we repeat the calculation for the initial steering angle, except we use the lap points, instead of the goal points. We finally have the steering angle. 

	alpha = self.find_alpha(lap[0], lap[1])
    angle = self.calculate_steering_angle(alpha) # steer to LAP

Now we write a **message** to sent to the motors, called **driver_msg**. This is an object that contains a variable for steering angle and acceleration. We will assign our steering angle to the variable. 

	drive_msg = AckermannDriveStamped()
    drive_msg.drive.steering_angle = angle

## Step 3 : Publish the Acceleration

The acceleration is computed in a separate function outside of self.cb(). It's called in self.wheel_speed_cb(), another callback function. Here though, the current acceleration (a global variable) is added to the drive_msg message. 

	drive_msg.drive.acceleration = self.current_accel

## Step 4: Publish Message

We're at the end of self.cb(), where we publish the drive_msg

## Other : Convert to Quaternions 