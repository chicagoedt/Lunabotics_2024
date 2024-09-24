The control algorithm is composed by different nodes, each with a specific function:

From the lowest to the highest level we have:

HARDWARE/SOFTWARE:
-Arduino node: it is the node responsible to communicate with Arduino, it receives in input a float value from -1 to 1 for all the motors controlled, 
it then commands the Arduino to give a PWM signal accordingly to the command given. From the Arduino takes the analog feedbacks and the motors angular 
velocities and returns them to the PID nodes (DONE)
-Camera node: it is the node responsible to take the signals coming from the camera and transform them to obtain

MIDDLEWARE:
-Motor Controller: receives in input the desired velocities to which actuate all the motors and return a float from -1 to 1 to the Arduino node
-PID nodes/linear actuators: the PID nodes implement a PID control to the motors by taking both the desired and real position of the actuators from the 
Arduino feedbacks and the input blocks
-PID nodes/motors: implement a PID controller on the velocities of the motors by taking into account both the desired and real velocities obtained by the 
input blocks and the Arduino feedbacks
-MAP planning : node that uses the Isaac ROS libraries to obtain a map of the environment
-VSLAM node(probable) : node that will take input from the camera and create a feedback position of the robot  

CONTROL LOGIC:
TRAJECTORY ALGORITHM: it gets a point to be reached from the main control logic and it keeps working with the map planning node to reach that point
it gives to the PID nodes the velocities to be given to the two wheels returns a done signals once the point is reached
DIGGING and DISCHARGING ALGORITHM: it gets a modality to be performed by the main control logic and controls the positions of the actuators to obtain
that modality, it returns done to the main control logic once the modality is finished. It has three modalities: initial position, digging mode and 
discharging mode 
MAIN CONTROL LOGIC: it is a long python scripts that takes care of the mission to be performed, it works by reaching goals: the first goal 
is to reach the digging area, therefore it will give a position to the trajectory algorithm to achieve, once the digging area is reached there will be a 
while loop to dig, go to the berm area, discharge and return to the digging area, it will talk to the map planning node to plan the best point to be reached
each time 