Assuming you read the Velocity Control and PID guide, then you know that the parameters (kp, ki and kd) need to be evaluated before been put into the code. There are many ways to tune a PID, and in this guide, we'll go over the methods we used and had success with. 

## Manual 

This is the simplest but tedious method of tuning. Here we make up a set of PID parameters, and observe how the car behaves. Depending on the behaviour, you can determine which parameter needs to be changed and by how much. Here is some things to watch out for:

- If the car is accelerating too fast, overshoots its setpoint by a large error. Then kp is too high and needs to be reduced.
- If the car overshoots and oscillates indefinitely, then ki may be too large, and needs to be reduced. 
- If the car takes too long to reach its setpoint, then increase kp.
- If the car takes a while to stabilize around the setpoint, then increase the ki.  

This method is very trial and error, which may be annoying. However, it doesn't require special knowledge of the system, you can just keep tweaking it until it's 'just right'. 

## System Identification 

This is a more sophisticated method of tuning, that will give the exact PID parameters. Here you will need MATLAB, which you can download for free (as a student) or access with the college computers. If you download MATLAB, you need to install the PID and System Identification toolboxes. 

### Prepare the Data
System identification works by inferring a model of the system from experimental or simulation data. This data can be obtained from simulating the car driving and recording the velocity over time, or from the rosbag containing the velocity of the FSUK vehicle. 


## Generate Model


## Tune PID
