#include "vel_optimal.h"
#include "math.h"

COMMAND	VSET[VSET_NUM] = { 0 };
OBSTACLE obs[OBS_SENSOR] = { 0 };

POSITION    target_pos(int dis_L, int dis_R) //get target position
{
	int		x, y, temp;
	int 	d_line;
	float 	S, H, P, Ltheta, cosine, theta;
	float	L, R, D;
	POSITION pos;

//robot view triangle frame
	L = dis_L * 1.0;
	R = dis_R * 1.0;
	D = DIS * 1.0;

//	sprintf( up2pc, "L:%lf R:%lf D:%lf ", L, R, D );
//	sonar_to_pc( up2pc, sizeof(up2pc) );

	P = (L + R + D) / 2;
	S = sqrt( P * (P-L) * (P-R) * (P-D) );
	H = 2 * S / D;
	cosine = (L*L + D*D - R*R)/(2.0*L*D);
	Ltheta = acos(cosine);

//	sprintf( up2pc, "P:%lf S:%lf H:%lf cos: %lf theta: %lf\n", P, S, H, cosine, Ltheta*180/3.14 );
//	sonar_to_pc( up2pc, sizeof(up2pc) );

	x = L * cosine - D/2.0;
	y = H;
	d_line = sqrt(x*x + y*y);

//	sprintf( up2pc, "X:%d Y:%d d_line:%d \n", x, y, d_line );
//	sonar_to_pc( up2pc, sizeof(up2pc) );
	//translate to robot frame
	temp = x;
	x = y;
	y = -1 * temp;
	theta = atan(1.0*y/x);

	pos.x = x;
	pos.y = y;
	pos.theta = theta;

	return pos;
}

STATUS		initial_vset(COMMAND *set)	//initial speed set
{
	int i = 0, j = 0;
	float vset[V_NUM], wset[W_NUM];

#ifdef DEBUG
	FILE	*fp;
	fp = fopen("VSET.txt", "w+");
#endif // DEBUG


	for (i = 0; i < W_NUM; i++)		//initial_wset=[-90,-85,...0,5,...,85,90], num = 37
		wset[i] = -90 + 5 * i;
	for (i = 0; i < V_NUM; i++)		//initial_vset=[0,0.05,...,0.95,1.00], num = 21
		vset[i] = 0.05*i;
	for (i = 0; i < W_NUM; i++)		//initial_VSET=[]
	{
		for (j = 0; j < V_NUM; j++)
		{
			set[21 * i + j].w = wset[i];
			set[21 * i + j].v = vset[j];
#ifdef DEBUG
			//printf("set[%d].w=%f  set[%d].v=%f\n", 21 * i + j, wset[i], 21 * i + j, vset[j]);
			fprintf(fp, "set[%d].w=%f  set[%d].v=%f\n", 21 * i + j, wset[i], 21 * i + j, vset[j]);
#endif
		}
	}

#ifdef DEBUG
	fclose(fp);
#endif // DEBUG

	return TRUE;
}
//input:  2 position p1: last_pos, p2: cur_pos
//output: vel & orient
STATE predictor(POSITION p1, POSITION p2)	//predict the target state: speed and orient
{
	STATE sta;
	float vel;
	float ort;
	float dx;
	float dy;

	dx = p2.x - p1.x;
	dy = p2.y - p1.y;

	vel = sqrt( dx*dx + dy*dy ) / CONTROL_PERIOD;
	if (dx > 0)
	{
		ort = atan2(dy, dx);
	}
	else
	{
		ort = PI / 2 + atan2(dy, dx);
	}
	sta.velocity = vel;
	sta.orient = ort;
	return sta;

}
/*
input: p1 = target last position, p2 = target current position
		obs = obstacle position
*/
COMMAND		optimal_vel( POSITION p1, POSITION p2, POSITION obs)
{

	POSITION	current_target, future_target, current_obs;
	POSITION	future_robot;
	STATE		target_state;
	COMMAND		cmd;
	float		current_gama, future_current, delta_theta, current_beta;

	current_target = p2;				//current & future target position
	target_state = predictor(p1, p2);
	future_target.x = current_target.x + target_state.velocity*T*cos(target_state.orient);
	future_target.y = current_target.y + target_state.velocity*T*sin(target_state.orient);

	current_obs = obs;					//get current obstacle position

	
	//update cost
	update_index(VSET, current_target, future_target, current_obs);

	//give a certain command, caculate the optimal index
	//for item in VSET:
	//	item.cost = index_cal( item,  )
	

	//if there is no obstacle, the current visible angle is positive
#ifdef DEBUG
	printf("x = %f y= %f\n", future_target.x, future_target.y);
#endif // DEBUG

	return find_optimal(VSET);
}

COMMAND		find_optimal(COMMAND *cmd)		//找到最优解
{
	COMMAND pos_result, neg_result;
	int i = 0;

	pos_result.v = 0;
	pos_result.w = 0;
	pos_result.index = 0; 

	neg_result.v = 0;
	neg_result.w = 0;
	neg_result.index = -1000000;

	for (i = 0; i < VSET_NUM; i++)
	{
		if (cmd[i].index > 0)
		{
			if (cmd[i].index > pos_result.index)
				pos_result = cmd[i];
		}
		else
		{
			if (cmd[i].index > neg_result.index)
				neg_result = cmd[i];
		}
	}

	if (neg_result.index != 0)
		return neg_result;
	else
		return pos_result;
	
}
//calculate speed command
STATUS		index_cal(COMMAND *cmd, POSITION target)
{
	POSITION future_robot;
	future_robot.x = cmd->v * T * cos(cmd->w);
	future_robot.y = cmd->v * T * sin(cmd->w);

	return TRUE;
}

//update the optimal index
STATUS		update_index(COMMAND *cmd, POSITION current_target, POSITION future_target, POSITION current_obs)
{
	int i;
	POSITION future_robot;
	float current_theta, future_theta, current_beta, future_beta, current_gama, future_gama;
	float delta_theta;

	current_gama = atan2(current_target.y, current_target.x);		//get the current target angel gama
	current_beta = atan2(current_obs.y, current_obs.x);				//obstacle angel: beta

	/*get current theta*/
	if (current_gama > 0)		//if target is in the left flat
	{
		if ((current_beta) > vision)	//if obstalce is outside of sensor vision 
		{
			current_theta = vision - current_gama;
		}
		else
		{
			current_theta = current_beta - current_gama;
		}
	}
	else						//right flat
	{
		if (current_beta < -1 * vision)
		{
			current_theta = vision - current_gama;
		}
		else
		{
			current_theta = current_beta - current_gama;
		}
	}
	
	/* caculate future theta and delta_theta */
	/* the maxsium negative delta_theta means theta is growing rapidly
	   the maxsium positive delta_theta means theta is decreasing slowly
	*/
	for (i = 0; i < VSET_NUM; i++)
	{
		if (cmd[i].w != 0)		//update future robot's position caused by speed command
		{
			future_robot.x = cmd[i].v * T * cos(cmd[i].w / 180.0*3.14);
			future_robot.y = cmd[i].v * T * sin(cmd[i].w / 180.0*3.14);
		}
		else
		{
			future_robot.x = cmd[i].v * T;
			future_robot.y = 0;
		}
		future_gama = atan2(future_target.y - future_robot.y, future_target.x - future_robot.x);
		future_beta = atan2(current_obs.y - future_robot.y, current_obs.x - future_robot.x);

		if (future_gama > 0)		//if target is in the left flat, here is the opposite to upside,
		{							// just handle left flat or right flat mark?
			if ((future_beta) > vision)
			{
				future_theta = vision - future_gama;
			}
			else
			{
				future_theta = future_beta - future_gama;
			}
		}
		else						//right flat
		{
			if (future_beta < -1 * vision)
			{
				future_theta = vision - future_gama;
			}
			else
			{
				future_theta = future_beta - future_gama;
			}
		}
		delta_theta = current_theta - future_theta;
		if (delta_theta != 0)
		{
			cmd[i].index = current_theta / delta_theta;
		}
		else
		{
			cmd[i].index = -0.000001;		//define the index that indicating no change for theta
		}


	}//for interator
	return TRUE;

}

//更新代价,无障碍物
STATUS		update_cost_noObs(COMMAND *cmd, float current_theta, POSITION future_target)
{
	int i;
	POSITION future_robot;
	float future_gama, future_theta, R, fai;

	for (i = 0; i < VSET_NUM; i++)
	{
		
		if (cmd[i].w != 0)
		{
			fai = cmd[i].w * T;
			R = cmd[i].v / cmd[i].w;		//motion radius
			future_robot.x = R*cos(fai / 180.0*PI);
			future_robot.y = R*sin(fai / 180.0*PI);
			future_theta = atan2(future_robot.y - future_target.y, future_robot.x - future_target.x) - cmd[i].w*T;
			cmd[i].index = current_theta - future_gama;
		}
		else
		{
			R = cmd[i].v * T;
			future_robot.x = R;
			future_robot.y = future_robot.y;
			future_gama = future_gama = atan2(future_robot.y - future_target.y, future_robot.x - future_target.x) - cmd[i].w*T;
			cmd[i].index = current_theta - future_gama;
		}
		
	}
	return TRUE;
}

//update optimal index when there is obstalce around
STATUS		update_cost_Obs(COMMAND *cmd, float current_theta, POSITION future_target, POSITION current_Obs)
{
	int i;
	POSITION future_robot,future_obs;
	float future_gama, future_beta, R, fai;

	for (i = 0; i < VSET_NUM; i++)
	{

		if (cmd[i].w != 0)
		{
			fai = cmd[i].w * T;
			R = cmd[i].v / cmd[i].w;		//motion radius
			future_robot.x = R*cos(fai / 180.0*PI);
			future_robot.y = R*sin(fai / 180.0*PI);
			future_beta = (current_Obs.y - future_robot.y, current_Obs.x - future_robot.x) - cmd[i].w * T;
			if (future_beta > vision)
			{
				future_gama = atan2(future_robot.y - future_target.y, future_robot.x - future_target.x) - cmd[i].w*T;
			}
			else
			{
				
			}
			
			cmd[i].index = current_theta - future_gama;
		}
		else
		{
			R = cmd[i].v * T;
			future_robot.x = R;
			future_robot.y = future_robot.y;
			future_gama = future_gama = atan2(future_robot.y - future_target.y, future_robot.x - future_target.x) + cmd[i].w*T;
			cmd[i].index = current_theta - future_gama;
		}

	}
	return TRUE;

}

COMMAND		find_max(COMMAND *cmd)
{
	COMMAND result;
	int i;
	result = cmd[0];
	for (i = 0; i < VSET_NUM; i++)
	{
		if (result.index < cmd[i].index)
		{
			result = cmd[i];
		}
	}
	return result;
}

STATUS		initial_map(MAP *map)
{
	map->width = MAP_HORIZON;
	map->height = MAP_VERTICAL;
	map->grid[MAP_HORIZON][MAP_VERTICAL] = { VACANT };
}

STATUS		update_map(int sensor[OBS_SENSOR], MAP* map)
{
	int width, height, index;
	float radius;
	float detect_vision;
	int w, h, arc;

	initial_map(map);

	detect_vision = 15.0 /180.0 * PI;
	for(index=0; index<OBS_SENSOR; index++)
	{
		width = sensor[index] / GRID_WIDTH;
		height = sensor[index] / GRID_HEIGHT;
		arc = sensor[index] * detect_vision;
	}
		

}