#ifndef		__GLOBAL_H__
#define		__GLOBAL_H__

#include	 "stdio.h"
#include 	 "stdlib.h"
#include 	 "pthread.h"
#include	 "vel_optimal.h"

extern 			pthread_mutex_t 	m;					//sych mutex
extern volatile int					sensor_frame[15]; 	//sensor data
extern volatile STATUS 				frame_update;		//sensor data finish flag
extern 			MAP					map;				//obstacle distribute map



extern volatile int 	test;

#endif