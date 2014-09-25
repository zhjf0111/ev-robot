#include "global.h"
#include "vel_optimal.h"


pthread_mutex_t 	m;	//sych mutex
volatile int		sensor_frame[15] = {0}; 	//sensor data
volatile STATUS 	frame_update = FALSE;		//sensor data finish flag
MAP		map = {0};						//map defined

volatile int 	test = 0;