#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <float.h>

#define INVALID 	-1
#define TRIGGER_GO 	0
#define LED_GO		1
#define WRITE_PERIOD	2
#define SIZE 		2



int change_blink(double avg){

		int blk_period=0;

		if(avg>100.0){
		blk_period=2000;
		}
		else if (avg>= 75.0 && avg<=100.0){
		blk_period=1000;
		}
		else if (avg>= 50.0 && avg<75.0){
		blk_period=800;
		}
		else if (avg>= 25.0 && avg<50.0){
		blk_period=600;
		}
		else if (avg>= 10.0 && avg<25.0){
		blk_period=400;
		}
		else if (avg<10.0){
		blk_period=0;
		}
		return blk_period;


}

int main(int argc, char **argv)
{
	char *app_name = argv[0];
	char *dev_name = "/dev/sample";
	int fd = -1;
	unsigned long long time;
	int x,i=0;
	int trigger_period = 1;
	int on = 0;
	double  distance=0;
	int blk_period=0;
	double avg=0;
	double buffer[SIZE];

	

	
	/*
 	 * Open the sample device RD | WR are allowed
 	 */
	if ((fd = open(dev_name, O_RDWR)) < 0) {
		fprintf(stderr, "%s: unable to open %s: %s\n", 
			app_name, dev_name, strerror(errno));
		return( 1 );
	}

	//Starting trigger 
	ioctl(fd,TRIGGER_GO,NULL);			    	
	x = write(fd, &trigger_period, sizeof(trigger_period)); 	

	
	//Starting led			
	ioctl(fd,LED_GO, NULL);	
	x = write(fd, &blk_period, sizeof(blk_period)); 
			
		

	while(1){

		
		do{
		x = read(fd, &time, sizeof(time));			//Reading echo value
		}while(time==0);
		
		
		distance = (double)(0.001*(time)/58);
		
		
		
		if(distance>=2.0 && distance<= 450.0){	 //if it was in range

			buffer[i]=distance;
			avg+=buffer[i];
			i++;

				if(i==SIZE){
				avg=avg/SIZE;
							
									
				printf("Distance: %lf \n",avg);
				blk_period=change_blink(avg);
				
				i=0;
				avg=0; 
			}
		
				//Update blink period
				ioctl(fd, WRITE_PERIOD, NULL);
				x = write(fd, &blk_period, sizeof(blk_period));

		
		}
		
	}		
		
	
	if (fd >= 0) 
	{
		close(fd);
	}
	return( 0 );
}
