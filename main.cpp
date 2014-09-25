#include "stdio.h"
#include "math.h"
#include "pthread.h"
#include "unistd.h"
#include "vel_optimal.h"
#include "serialport.h"
#include "process.h"
#include "stdlib.h"
#include "global.h"

int main()
{

	pthread_t t1;
    pthread_t t2;

	initial_map(&map);	
	/*
    pthread_create(&t1,NULL,data_decode,NULL);
    pthread_create(&t2,NULL,processB,NULL);
  //  pthread_join(t1, NULL);
    pthread_join(t1, NULL);
    pthread_join(t1,NULL);
*/
	/*
	while(1)
  	{
    	if((nread = read(fd, buff, 512))>1)
	    {
	      printf("\nLen: %d\n",nread);
	      buff[nread+1] = '\0';
	      for(i=0; i<nread; i++)
	      printf("%c", buff[i]);
	      printf("\n");
	      memset(buff, 0, sizeof(buff));
	      printf("CNT = %d\n", cnt++);
	    }

  	}
	*/

	
	/*
	POSITION p1, p2;
	POSITION obs;
	STATE result;

	obs.x = 1.3;
	obs.y = 0.3;

	p1.x = 0;
	p1.y = 0;
	p2.x = 1;
	p2.y = -1*sqrt(3.0);
	result = predictor(p1, p2);
	printf("orient = %f  vel = %f\n", result.orient/PI*180, result.velocity);
	optimal_vel(p1, p2, obs);

	initial_vset(VSET);
	*/

	return 0;
}