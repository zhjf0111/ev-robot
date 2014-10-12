ev-robot
========
Control method on tracking & following based on ultrasound sensors.

--vel_optimal:
	add obstacle detection function. the function should scan all the data that from all environment detection sensor, 
	return the position and shape information of the obstacle.

	A map is defined to help caculate obstacle information. Grip: 100mm x 100mm Map: 60 x 30
	Sensor's data is mapped to grid location.

 	Thus a struct that describes an obstacle should be defined. 
	
	According to the sensor data, obstalce is described in the map as grid point.

	Next to do is to list the process to find optimal vel and complete other codes.