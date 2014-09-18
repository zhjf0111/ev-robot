#ifndef		__SERIALPORT_H__
#define		__SERIALPORT_H__


#include<stdio.h>
#include<stdlib.h>
#include<unistd.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<termios.h>
#include<errno.h>


#define		H1		0
#define		H2 		1
#define 	E1		32
#define		E2 		33

extern int speed_arr[15];
extern int name_arr[15]; 

int 	openserial(char port[], int speed, int databits, int parity, int stopbits );
void 	set_speed(int fd, int speed);
int 	set_Parity(int fd,int databits,int parity,int stopbits);

#endif