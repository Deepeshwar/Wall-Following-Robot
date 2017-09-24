/*
 * Wall_Following_V1.c
 *
 * Created: 4/11/2017 5:15:09 AM
 *  Author: Deepeshwar Kumar
 */ 


#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <avr/interrupt.h>
#include "USART_128.h"

#define Angle 45 

volatile uint16_t i,j, dis[5];
volatile long left_rpm, right_rpm;
volatile long left_tx,right_tx;
float RightTheta,LeftTheta,left_w,Right_w;
volatile float kp[2] = {10,10}, ki[2] = {0, 0}, kd[2] = {0, 0}, E[2] = {0, 0}, e_old[2] = {0, 0}; 

void pwm() {
	TCCR3A |= 1<<WGM31 | 1<<COM3A1| 1<<COM3B1;
	TCCR3B |= 1<<WGM32  | 1<<WGM33 | 1<<CS31;
	ICR3=30000;
}



 float PID(float error,int x)
 {
	 float pid = 0;
	 pid = (kp[x]*error) + (ki[x]*E[x]) + (kd[x]*(error - e_old[x]));
	 E[x]+=error;
	 e_old[x] = error;
	 return pid;
 }
 
void Get_WallBot_Angles(float *RightTheta,float *LeftTheta)
{
	 *RightTheta = atan2(dis[1] - (dis[0]/cos(Angle)),dis[0]*tan(Angle));
	 *LeftTheta  = atan2(dis[3] - (dis[4]/cos(Angle)),dis[4]*tan(Angle));
}

int main(void)
{
	/*pwm();
	sei();
	USART_Init(12,0);
	//USART_Init(12,1);
	USART_InterruptEnable(0);
	DDRD|=0xff;
	DDRE|=0xff;
	*/
	
	DDRE |= (1<<PINE2) | (1<<PINE3) | (1<<PINE4) | (1<<PINE5);
	//DDRD |= 0XFF;
	
	//interrupt , any logical change
	EICRA |= (1<<ISC20) | (1<<ISC21)| (1<<ISC30) | (1<<ISC31);
	EIMSK |= (1<<INT2) | (1<<INT3);
	
	//timers 10 bit
	TCCR0 |= (1<<CS02) | (1<<CS01) | (1<<CS00);
	TIMSK |= (1<<TOIE0);
	
	//PWM_timer , Fast_PWM_mode, Top = 0x03FF(in Hex) or 1023(in Decimal)
	TCCR3B |= (1<<CS30) | (1<<WGM32);
	TCCR3A |= (1<<COM3A1) | (1<<COM3B1) | (1 << WGM31) | (1<< WGM30);
	
	//init_HMC5883L();
	
	USART_Init(12,0);
	//USART_Init(12,1);
	USART_InterruptEnable(0);
	
		

    while(1)
    {
		Get_WallBot_Angles(&RightTheta,&LeftTheta);
		
		Right_w = PID(-RightTheta,0);
		left_w = PID(-LeftTheta,1);
		
		left_wall_following();
		Right_wall_following();
		USART_Transmitchar('R',0);
		USART_TransmitNumber(RightTheta,0);
		USART_Transmitchar('L',0);
		USART_TransmitNumber(LeftTheta,0);
		USART_Transmitchar(0x0D,0);
    }
}

void left_wall_following()
{

	if(LeftTheta>=-1 && LeftTheta<=1)
	{
		OCR3A=8000;
		OCR3B=8000;
		
		left_tx=8000;
		right_tx=8000;
		
	}
	else
	{
		left_rpm=5000+left_w;
		right_rpm=5000-left_w;
		
		if(right_rpm>8000) right_rpm=8000;
		if(left_rpm>8000) left_rpm=8000;
		left_tx = left_rpm;
		right_tx = right_rpm;
		
		if(right_rpm<0)
		{
			PORTE|=(1<<PINE5);
			right_rpm=-right_rpm;

		}
		else{
			PORTE&=~(1<<PINE5);
		}
		
		if(left_rpm<0)
		{
			PORTE|=(1<<PINE2);
			left_rpm=-left_rpm;
		}
		else{
			PORTE&=~(1<<PINE2);
		}
		
		if(right_rpm>8000) right_rpm=8000;
		if(left_rpm>8000) left_rpm=8000;
		
		
		OCR3A=left_rpm;
		OCR3B=right_rpm;
		
	}
	
}

void Right_wall_following()
{

	if(RightTheta>=-1 && RightTheta<=1)
	{
		OCR3A=8000;
		OCR3B=8000;
		
		left_tx=8000;
		right_tx=8000;
		
	}
	else
	{
		left_rpm=5000-Right_w;
		right_rpm=5000+Right_w;
		
		if(right_rpm>8000) right_rpm=8000;
		if(left_rpm>8000) left_rpm=8000;
		left_tx = left_rpm;
		right_tx = right_rpm;
		
		if(right_rpm<0)
		{
			PORTE|=(1<<PINE5);
			right_rpm=-right_rpm;

		}
		else{
			PORTE&=~(1<<PINE5);
		}
		
		if(left_rpm<0)
		{
			PORTE|=(1<<PINE2);
			left_rpm=-left_rpm;
		}
		else{
			PORTE&=~(1<<PINE2);
		}
		
		if(right_rpm>8000) right_rpm=8000;
		if(left_rpm>8000) left_rpm=8000;
		
		
		OCR3A=left_rpm;
		OCR3B=right_rpm;
		
	}
	
}

ISR(USART0_RX_vect)
{
	char distance;
	
	distance=USART_Receive(0);
	
	if(distance == 'A')
	{
		j=0;
		dis[0]=0;
	}
	else if (distance == 'B')
	{
		j=1;
		dis[1]=0;
	}
	else if (distance == 'C')
	{
		j=2;
		dis[2]=0;
	}
	else if (distance == 'D')
	{
		j=3;
		dis[3]=0;
	}
	else if (distance == 'E')
	{
		j=4;
		dis[4]=0;
	}
	else
	{
		
		dis[j]=dis[j]*10 + (distance-'0');
	}
	
}