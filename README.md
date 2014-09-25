ev-robot
========
Control method on tracking & following based on ultrasound sensors.

--vel_optimal:
	add obstacle detection function. the function should scan all the data that from all environment detection sensor, 
	return the position and shape information of the obstacle.

	A map is defined to help caculate obstacle information. Grip: 100mm x 100mm Map: 60 x 30
	Sensor's data is mapped to grid location.

 	Thus a struct that describes an obstacle should be defined 
	first. The struct should consist of 
	Sencond, the detector should 
	obstacle_detect(int sensor[obs_sensor], )