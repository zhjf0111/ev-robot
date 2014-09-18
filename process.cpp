#include "process.h"
#include "serialport.h"
#include "pthread.h"
#include "global.h"
#include "vel_optimal.h"

void* data_decode(void* argv)
{
	int cnt = 0, nread = 0;
	int state = 0;
	STATUS finish = FALSE;
	int fd;
	int timer = 0;
	unsigned char frame[128], buf[512];
	fd = openserial((char*)"/dev/ttyUSB0", 19200, 8, 'N', 1);
	if(fd == -1)
	{
		printf("open ttyUSB0 failed!\n");
		//return;
	}
	else
	{
		//printf("open ttyUSB0 success!\n");
		while(1)
		{
			//printf("in while...\n");
		    if((nread = read(fd, buf, 512))>1)
		    {
		    	//printf("data receive! nread = %d\n", nread);
		    	for(int i=0; i<nread; i++)
		    	{
		    		//printf("%c", buf[i]);
		    		switch(state)
			    	{
			    		case H1:
			    			if( 'S' == buf[i] )
			    			{
			    				state++;
			    				//printf("S");
			    			}
			    			else
			    				state = 0;
			    			break;
			    		case H2:
			    			if( 'J' == buf[i] )
			    			{
			    				state++;
			    				//printf("J");
			    			}
			    				
			    			else
			    				state = 0;
			    			break;
			    		case E1:
			    			if( 'T' == buf[i] )
			    			{
			    				//printf("T");
			    				state++;
			    			}
			    				
			    			else
			    				state = 0;
			    			break;
			    		case E2:
			    			if( 'U' == buf[i] )
			    			{
			    				//printf("U\n");
			    				//state ++;
			    				finish = TRUE;
			    			}
			    			else
			    				state = 0;
			    			break;
			    		default:

			    			frame[state - 2] = buf[i];
			    			state++;
			    			break;

			    	} 

			    	 if ( TRUE == finish )
				    {
				    	pthread_mutex_lock(&m);
				    	for(int k=0; k<15; k++)
				    	{
				    		sensor_frame[k] = frame[2*k] * 256 + frame[2*k+1];
				    	}
				    	frame_update = TRUE;
				    	finish = FALSE;
				    	state = 0;
				    	pthread_mutex_unlock(&m);

				    	/*printf("sensor_frame update... timer: %d\n", timer++);
					    for(int k=0; k<15; k++)
					    {
					    	printf("%d ", sensor_frame[k]);
					    }
					    printf("\n");*/
				    }
		    	}
		    	
		    }

		    

  		}
  			close(fd);
			printf("Process A is here!\n");
			test = cnt++;
			frame_update = TRUE;
			sleep(1);
	}
	
}

void* processB(void* argv)
{
	int sensor[15];
	STATUS copy = FALSE;
	POSITION target_pos_current;
	POSITION target_pos_last;
	POSITION obstacle_pos;
	STATE target_state;

	initial_vset(VSET);

	while(1)
	{
		pthread_mutex_lock(&m);//copy data to local
		if(1 == frame_update)
		{
			for(int i=0; i<15; i++)
			{
				sensor[i] = sensor_frame[i];
			}
			frame_update = FALSE;
			copy = TRUE;
		}
		pthread_mutex_unlock(&m);
		if(TRUE == copy)
		{
			printf("after copy...\n");
			for(int i=0; i<15; i++)
			{
				printf("%d ", sensor[i]);
			}
			printf("\n");
			copy = FALSE;
			//target_pos_last = target_pos_current;
			//target_pos_current = target_pos(sensor[13], sensor[14]);
			//printf("target pos: x=%f y=%f theta=%f\n", target_pos_current.x, target_pos_current.y, target_pos_current.theta);
			//belows are for testing
			target_pos_last.x = 500;
			target_pos_last.y = 500;
			target_pos_current.x = 1000;
			target_pos_current.y = 1000;
			target_state = predictor(target_pos_last, target_pos_current);
			printf("target_state: vel = %f ort =%f \n", target_state.velocity, target_state.orient*180/3.14);
		}

	}
}