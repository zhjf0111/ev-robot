ev-robot
========
Control method on tracking & following based on ultrasound sensors.

--main process flow:
	1. caculate target position & state(store the position & state in each control period) and target future position, if there is no target, jump to 5
	2. construct the grid map with obstacle information and get obstacle position
	3. use the discrete veloctiy set, caculate each command's influence on vision ability and store it in cmd.index
	4. find the optimal cmd and send it to robot
	5. remeber the target's last seen position and state, predict him continuesly
	6. the target's predition position is attracted force and obstalce is repulsive fore, caculate the combined command and send it to robot
--vel_optimal:
	add obstacle detection function. the function should scan all the data that from all environment detection sensor, 
	return the position and shape information of the obstacle.

	A map is defined to help caculate obstacle information. Grip: 100mm x 100mm Map: 60 x 30
	Sensor's data is mapped to grid location.

 	Thus a struct that describes an obstacle should be defined. 
	
	According to the sensor data, obstalce is described in the map as grid point.

	Next to do is to list the process to find optimal vel and complete other codes.

