#ifndef __VEL_OPTIMAL_H__
#define	__VEL_OPTIMAL_H__

#include "stdio.h"
//#include "map"

//using namespace std;
//#define		DEBUG				1

/* control parameter define */
#define		CONTROL_PERIOD		0.1
#define		T					CONTROL_PERIOD
#define		PI					3.141592653

/* robot parameter define */
#define		vMax				1.0
#define		wMax				90
#define		vision				45		//sensor vision coverage, Degree

/* speed parameter define */
#define		V_NUM				21
#define		W_NUM				37
#define		VSET_NUM			V_NUM * W_NUM

/* sensor parameter define */
#define 	DIS					180		//define the dis between tracking sensors 
#define 	OBS_SENSOR			13
#define		TRAC_SENSOR			2
#define		ALL_SENSOR			OBS_SENSOR+TRAC_SENSOR


typedef struct 
{
	float velocity;
	float orient;
}STATE;			//velocity: m/s	orient: rad

typedef struct
{
	float v;
	float w;
	float index;
}COMMAND;

typedef struct 
{
	float x;
	float y;
	float theta;
}POSITION;		//x,y : m	theta: rad

typedef struct 
{
	POSITION p1;
	POSITION p2;
}OBSTACLE;

typedef enum{FALSE = 0, TRUE = 1}STATUS;

extern	COMMAND	VSET[VSET_NUM];
extern  OBSTACLE obs[OBS_SENSOR];

/* map and obstacle parameter define */
#define 	GRID_WIDTH			100 		//milimeter
#define 	GRID_HEIGHT			100 		//milimeter

#define 	MAP_HORIZON 		6000/GRID_WIDTH
#define 	MAP_VERTICAL		3000/GRID_HEIGHT

typedef enum{VACANT = 0, OCCUPIED = 1}GRID;
typedef struct 
{
	int 			width;
	int 			height;
	GRID 			grid[MAP_VERTICAL][MAP_HORIZON];
	int 			grid_center[MAP_VERTICAL][MAP_HORIZON];

}MAP;




POSITION    target_pos(int s1, int s2);  //input: sensor data, output: target position

//get the obstalce information, return sensor index that has detects
//obstacles
STATUS		initial_map(MAP *map);								//initial map
STATUS		clear_map(MAP *map);								//clear map
STATUS		update_map(int sensor[OBS_SENSOR], MAP* map);		//according to the sensor data, construct obstacle distribution map	
void 		draw_map(POSITION p1, POSITION p2, int radius, MAP* map);		//according to sensor data & obstalce data, draw information in map
STATUS		initial_vset(COMMAND *set);
STATE		predictor(POSITION p1, POSITION p2);				//predict the next state for the target

//如果没有障碍物，当前可视角为正角度
//input:	target position	
//output:	optimalized vel cmd
COMMAND		optimal_vel(POSITION p1, POSITION p2, POSITION obs);								//

//给定速度指令，计算其评价指标
STATUS		index_cal(COMMAND *cmd, float current_theta);
STATUS		update_index(COMMAND *cmd, POSITION current_target, POSITION future_target, POSITION current_obs);	//已知当前目标,预测目标，和当前障碍物的位置关系

STATUS		update_cost_noObs(COMMAND *cmd, float current_theta, POSITION future_target);
STATUS		update_cost_Obs(COMMAND *cmd, float current_theta, POSITION future_target, POSITION current_Obs);
COMMAND		find_max(COMMAND *cmd);
COMMAND		find_optimal(COMMAND *cmd);		//找到最优解
#endif