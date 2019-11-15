#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <math.h>
#include <pthread.h>
#include <unistd.h>

#define LED1 7
#define LED2 1
#define LED3 5
#define LED4 14


#define	MAG_ADD			0x1E
#define GYRO_ADD		0x68

#define MST_ON          wiringPiI2CWriteReg8(fd,0x6A,wiringPiI2CReadReg8(fd,0x6A) | 0x20)
#define MST_OFF         wiringPiI2CWriteReg8(fd,0x6A,wiringPiI2CReadReg8(fd,0x6A) & 0xDF)

#define BYPASS_ON       wiringPiI2CWriteReg8(fd,0x37,wiringPiI2CReadReg8(fd,0x37) | 0x02)
#define BYPASS_OFF      wiringPiI2CWriteReg8(fd,0x37,wiringPiI2CReadReg8(fd,0x37) & 0xFD)

#define col 2
#define row 1

int fd,fd2;
signed short mx,my,mz;
float heading;
float latitude, longitude;
float realative_error=0.000025;
char go = 0, action =1;
float goal_goal_degree;

//MAP CHECK POINT
float goal_x[] = { 36.351459777776667, 36.3511638889, 36.35117944443333, 36.35114555566667, }; //latitude
float goal_y[] = { 127.30034177776667, 127.30034177776667, 127.300338889, 127.301168889, 127.301448889 }; //longitude


void slv_write(char num, int reg_add,int data)
{
	wiringPiI2CWriteReg8(fd,0x25+num*3,0x1E);		// MSB -> Write : 0 read : 1    
	wiringPiI2CWriteReg8(fd,0x26+num*3,reg_add);	// SLV0_REG_Address
	wiringPiI2CWriteReg8(fd,0x27+num*3,0x81);		// SLV0_CONTROL
	wiringPiI2CWriteReg8(fd,0x63+num,data);		// SLV0_Data_Input
}
void slv_read(char num, int reg_add)
{
	wiringPiI2CWriteReg8(fd,0x25+num*3,0x1E|0x80);	// MSB -> read : 1
	wiringPiI2CWriteReg8(fd,0x26+num*3,reg_add);	// To Read REG
	wiringPiI2CWriteReg8(fd,0x27+num*3,0x82);		// SLV0_CONTROL
}
void MPU6050_INIT()
{

    wiringPiI2CWriteReg8(fd,0x6B,0x80);		// MPU6050 Reset

	wiringPiI2CWriteReg8(fd,0x1B,0x00);		// disable sleep mode!!
	wiringPiI2CWriteReg8(fd,0x6B,0x00);		// MPU6050 config
	wiringPiI2CWriteReg8(fd,0x6C,0x00);		// MPU6050 config	
}
void mag_init()
{
	char output_rate, error_rate;

	output_rate = 0x10; 
	// Data Output Rate  15Hz : 0x10 / 30Hz : 0x14 / 75Hz : 0x18
	error_rate = 0x60; 
	// Error Range  0.88Ga : 0x00 / 1.3Ga(default) : 0x20 / 1.9Ga : 0x40 /2.5Ga : 0x60 
	//               4.0Ga : 0x80 / 4.7Ga : 0xA0 / 5.6Ga : 0xC0 / 8.1Ga : 0xE0
	
	slv_write(0,0x00,output_rate); 			// Data Output Rate	
	slv_write(0,0x01,error_rate);			// 오차
}
void get_mag_data()
{

		MST_ON;
		slv_write(0,0x02,0x00);				// continuous (연속모드설정)
		slv_read(0,0x03);
		slv_write(1,0x02,0x00);				// continuous (연속모드설정)
		slv_read(1,0x05);	
		slv_write(2,0x02,0x00);				// continuous (연속모드설정)
		slv_read(2,0x07);	
		
		delay(10);
		mx = wiringPiI2CReadReg8(fd,0x49)<<8 |  wiringPiI2CReadReg8(fd,0x4A);	
		my = wiringPiI2CReadReg8(fd,0x4B)<<8 |  wiringPiI2CReadReg8(fd,0x4C);	
		mz = wiringPiI2CReadReg8(fd,0x4D)<<8 |  wiringPiI2CReadReg8(fd,0x4E);
}


float Where_is_the_goal(float goal_x,float goal_y,float my_x,float my_y)
{
	/////////////////////// Chul Code //////////////////
	// 변수추가
	float heading_x,heading_y;
	float heading_degree, final_degree;
	char area;
	
	

	// Where is the Goal ( display with degree )
	heading_x = goal_x - my_x ;
	heading_y = goal_y - my_y;


	if( heading_x >= 0 && heading_y >= 0 ) // area 1
		area = 1;				
	else if( heading_x < 0 && heading_y >= 0 ) // area 2
		area = 2;
	else if( heading_x < 0 && heading_y < 0 )	// area 3
		area = 3;
	else if( heading_x >= 0 && heading_y < 0 )  // area 4
		area = 4;	

	
	heading_x = fabs(heading_x);						// Abs heading
	heading_y = fabs(heading_y);


	heading_degree = atan2( heading_x,heading_y );		// Calculate Arctan(x,y);
	heading_degree = heading_degree *180 / M_PI;
	
	switch( area )
	{
		case 3:	heading_degree = 90 - heading_degree; break; 
		case 4:	heading_degree = 270 + heading_degree; break;
		case 1:	heading_degree = 180 + heading_degree; break;
		case 2: heading_degree = 90 + heading_degree; break;
	}
	
	return heading_degree;
}

char Check_heading(float goal_degree , float my_degree)
{
	float up_range_degree, down_range_degree ;
	char over_degree_flag = 0;
	char action_val;


	up_range_degree = goal_degree + 20;				// Up range
	down_range_degree = goal_degree - 20;			// Dn range

	if(up_range_degree > 360)							// if up_range > 360  then
	{

		up_range_degree = up_range_degree - 360;
		over_degree_flag = 1;
	}
	else if(down_range_degree < 0)						// if Down_range < 0 then
	{
		down_range_degree = down_range_degree + 360;	
		over_degree_flag = 2;
	}
	// over_degree_flag  0 : Normal Mode
	//					1 : Up_range 
	//					2 : Down_range
	//////////////////////////////////////////////////
	// Action 			1 : Nice Direction
	//					2 : Turn Right
	//					3 : Ture Left
	switch(over_degree_flag)
	{
		case 0:
			
			if( (down_range_degree < my_degree) && (up_range_degree > my_degree) )
			{
				action_val = 1;
			}

			break;
		case 1:					// if Over range
			if( (down_range_degree < my_degree) || (up_range_degree > my_degree))
			{
				action_val = 1;
			}
			break;
		case 2:					// if Down range
			if( (down_range_degree < my_degree) || (up_range_degree > my_degree) )
			{
				action_val = 1;
			}
			break;
	}

	/////////////////////////////////////////////

	if (action_val != 1)
	{

		if( my_degree - goal_degree <0 )
			action_val = 2;
		else
			action_val = 3;
	}

		/*
	if( my_degree + 180 >= 360 )
	{
		if( (my_degree < (goal_degree + 360)) && ((my_degree + 360) > goal_degree+180) )
			action_val = 2;
		else
			action_val = 3;
	}

	else
	{
		if( (my_degree < goal_degree) && (goal_degree < my_degree + 180))
			action_val = 2;
		else
			action_val = 3;
	}
	*/
	
	return action_val ;
}


void *head_func(void *data)
{
	float num = 2.0;
	int a = *((int *)data);
	get_mag_data();
		//printf("Mag_x : %4d / Mag_y : %4d / Mag_z : %4d / ",mx,my,mz);		
		heading = atan2(mz,my); 		// 정면으로 보고 납땜 위쪽

		if(heading < 0)
			heading += 2* M_PI;
		float b_heading = heading * 180/M_PI;
	while(1)
	{
		
		get_mag_data();
		//printf("Mag_x : %4d / Mag_y : %4d / Mag_z : %4d / ",mx,my,mz);		
		heading = atan2(mz,my); 		// 정면으로 보고 납땜 위쪽

		if(heading < 0)
			heading += 2* M_PI;
		heading = heading * 180/M_PI;
		
		heading = b_heading * ((num-1)/num) + (1/num)*heading;
		num++;
		b_heading = heading;
		if(num >=15)num = 2.0;
		//printf("heading : %f \r\n",heading);

		delay(80);
	}
	
}

void *led_func(void *data)
{
	int b = *((int *)data);
	while(1)
	{
		switch(action)
		{
			case 1:		// go straight
			
				break;
			case 2:		// turn right
				digitalWrite(LED3, 1);
				digitalWrite(LED4, 1);
				delay(2000);
				digitalWrite(LED3, 0);
				digitalWrite(LED4, 0);
				delay(500);
				break;

			case 3:		// turn left
				digitalWrite(LED1, 1);
				digitalWrite(LED2, 1);
				delay(2000);
				digitalWrite(LED1, 0);
				digitalWrite(LED2, 0);
				delay(500);
				break;

			case 4:
				digitalWrite(LED1, 1);
				digitalWrite(LED2, 1);
				digitalWrite(LED3, 1);
				digitalWrite(LED4, 1);
				delay(5000);
				digitalWrite(LED1, 0);
				digitalWrite(LED2, 0);
				digitalWrite(LED3, 0);
				digitalWrite(LED4, 0);
				delay(500);
	
				break;
			}		
	}
}
		

int main(int argc, char *argv[])
{
	double *DATA;
	char dum[4096];
	char seps[] = ", ";
	char *token;
	FILE *read1;
	float goal_degree;
	char a = 0, b = 0;
	
	wiringPiSetup();

	pinMode(LED1,OUTPUT);
	pinMode(LED2,OUTPUT);
	pinMode(LED3,OUTPUT);
	pinMode(LED4,OUTPUT);

	digitalWrite(LED1, 0);
	digitalWrite(LED2, 0);
	digitalWrite(LED3, 0);
	digitalWrite(LED4, 0);

	fd=wiringPiI2CSetup(GYRO_ADD);
	MPU6050_INIT();

	float s_num = 2.0;
	float b_latitude;
	float b_longitude;
	pthread_t p_thread[2];		
	pthread_create(&p_thread[0], NULL, head_func, (void *)&a);
	pthread_create(&p_thread[1], NULL, led_func, (void *)&b);
	
		DATA = (double *)calloc(col,10);
		read1 = fopen("position.txt","rt");

		fgets(dum, 4096, read1);
		token = strtok(dum, seps); DATA[1] = atof(token);
		token = strtok(NULL, seps); DATA[2] = atof(token);
				
		latitude = DATA[1];
		longitude = DATA[2];	
		
		b_latitude = latitude;
		b_longitude = longitude;

while(1){
		
		
		DATA = (double *)calloc(col,10);
		read1 = fopen("position.txt","rt");

		fgets(dum, 4096, read1);
		token = strtok(dum, seps); DATA[1] = atof(token);
		token = strtok(NULL, seps); DATA[2] = atof(token);
				
		latitude = DATA[1];
		longitude = DATA[2];	
		

		latitude = b_latitude * ((s_num-1)/s_num)+(1/s_num)*latitude;		
		longitude = b_longitude * ((s_num-1)/s_num)+(1/s_num)*longitude;
		s_num++;
		b_latitude = latitude;
		b_longitude = longitude;

		if(s_num >= 10)s_num = 2.0;
		printf("Heading : %f  /  Latitude : %f  /  Longitude : %f / goal : %f \r\n", heading, latitude, longitude,goal_goal_degree);
		action = 0;
		// arrive check code 	
		if(latitude + realative_error <= goal_x[go] && latitude - realative_error <= goal_x[go])
		{
			if(longitude + realative_error <= goal_y[go] && longitude - realative_error <= goal_y[go])
			{
				if(go<4)	
				{
					go++; // Next goal
					action =4;
				}
			}
		}
		goal_degree = Where_is_the_goal(goal_x[go],goal_y[go],latitude,longitude );

		goal_goal_degree = goal_degree;

		if ( action != 4)		// If arrive check point no execute
			action = Check_heading(goal_degree, heading);		
		// action  1: Ok direction / 2: Turn Right / 3: Turn Left / 4 : Arrive Check Point


		delay(100);

	
	
	
	
	}

	
	return 0;

}
