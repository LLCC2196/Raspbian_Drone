#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/time.h>
#include "rc.h"
#include "isr.h"
int rc_count = 0;
void rc_calculate(struct timeval start,struct timeval end)
{
	int time_use = 0;
	time_use=(end.tv_sec-start.tv_sec)*1000000+(end.tv_usec-start.tv_usec);	
//	printf("time_use:%d\n",time_use);

	if ((rc_count < 8)&(time_use>500)&(time_use<2000))
		{
		RC_RECIVED = 0;	
		RC[rc_count] = time_use+400;
//		printf("rc_count:%d\ttime_use:%d\n",rc_count,time_use);
		rc_count++;
//		RC_RECIVED = 0;	       
		}
	else if((rc_count>=8)&(time_use>2000))
		{
//		printf("rc-recived!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
		RC_RECIVED = 1;
		rc_count = 0;
/*
		int i;
		for (i = 0 ; i<8 ;i++)
		{
	        printf("RC%d : %d\t",i,RC[i]);
		}
		printf("\n");
*/
		}
	else 	
		{
//		printf("rc-recive-error!!!!!!!!!!!!!!!!!!!!\n");
		rc_count = 0;
		RC_RECIVED = 0;
		}

}
int RC_Init(void)
{
	PiSetup();
	PiISR(0, INT_EDGE_RISING,&Interrupt_INT_EDGE_RISING);
	PiISR(1, INT_EDGE_FALLING,&Interrupt_INT_EDGE_FALLING);
	return 0;
}
