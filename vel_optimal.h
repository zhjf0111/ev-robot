#ifndef __VEL_OPTIMAL_H__
#define	__VEL_OPTIMAL_H__

#include "stdio.h"

//#define		DEBUG				1

#define		CONTROL_PERIOD		0.1
#define		T					CONTROL_PERIOD
#define		PI					3.141592653

#define		vMax				1.0
#define		wMax				90
#define		vision				45		//sensor vision coverage

#define		V_NUM				21
#define		W_NUM				37
#define		VSET_NUM			V_NUM * W_NUM

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

POSITION    target_pos(int s1, int s2);  //input: sensor data, output: target position

//get the obstalce information, return sensor index that has detects
//obstacles
int 		obstacle_pos(int sensor[OBS_SENSOR], OBSTACLE *obs);	 	
STATUS		initial_vset(COMMAND *set);
STATE		predictor(POSITION p1, POSITION p2);		//predict the next state for the target

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