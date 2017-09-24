/*
 * Wall_Following.c
 *
 * Created: 4/11/2017 4:34:12 AM
 *  Author: Deepeshwar Kumar
 */ 

#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include "USART_128.h"

#define thresold     40
#define limit        20
#define kp			 200
#define kd			 100
#define ki			 0

void right_wall_following();
void left_wall_following();

unsigned char distance;
float pid_calc=0;
uint8_t j = 0;
volatile int32_t Sensor[3] = {0,0,0};
volatile float  mearsured_distance, error,last_err;
volatile long left_rpm, right_rpm;
volatile long left_tx,right_tx;

void pwm() {
	TCCR3A |= 1<<WGM31 | 1<<COM3A1| 1<<COM3B1;
	TCCR3B |= 1<<WGM32  | 1<<WGM33 | 1<<CS31;
	ICR3=20000;
}

int main(void)
{
	pwm();
	sei();
	USART_Init(12,0);
	USART_Init(12,1);
	USART_InterruptEnable(0);
	DDRG|=0xff;
	DDRB|=0xff;
	
  while(1)
    {

 		USART_TransmitNumber(Sensor[1],0);		//left	
  		_delay_ms(100);
  		USART_TransmitString("    ",0);
  		
		USART_TransmitNumber(Sensor[2],0);		//right
		_delay_ms(100);
		USART_TransmitString("    ",0);
 	 
		 PORTB&= ~(1<<PINB7);
		 PORTG&= ~(1<<PING3);
		 
		if(Sensor[0] > thresold && Sensor[1] > thresold && Sensor[2] > thresold)	
		 {
			 OCR1A=8000;
			 OCR1B=8000;
			 left_tx=OCR1A;
			 right_tx=OCR1B;
		 }
		 else if (Sensor[0] >thresold && Sensor[1] < thresold && Sensor[2] > thresold)
		 {
			 left_wall_following();	
			 USART_TransmitString("error",0);
			 USART_TransmitNumber(error,0);	
			 USART_TransmitString("	",0);			//wall following using left_sensor
		 }
		 else if (Sensor[0] > thresold && Sensor[1] > thresold && Sensor[2] <thresold)
		 {
			 right_wall_following();		
			 USART_TransmitString("error",0);
			 USART_TransmitNumber(error,0);	
			 USART_TransmitString("	",0);			//wall following using right_sensor
		 }
		 else if (Sensor[0] < thresold && Sensor[1] > thresold && Sensor[2] > thresold)
		 {
			 OCR1A=12000;
			 PORTB|=(1<<PINB7);
			 OCR1B=4000;
			 					
			 left_tx=OCR1A;			//right turn
			 right_tx=OCR1B;
			
		 }
		 else if (Sensor[0] < thresold && Sensor[1] < thresold && Sensor[2] > thresold)
		 {
			  OCR1A=12000;
			  OCR1B=4000;	
			  PORTB|=(1<<PINB7);		//right turn
			  left_tx=OCR1A;
			  right_tx=OCR1B;
			 
		 }
		 else if (Sensor[0] < thresold && Sensor[1] >thresold && Sensor[2] < thresold)
		 {
			 PORTG|=(1<<PING3);
			 OCR1A=4000;
			 OCR1B=12000;			 //left turn
			 left_tx=OCR1A;
			 right_tx=OCR1B;
			 
					
		 }
		  else if (Sensor[0] < thresold && Sensor[1] <thresold && Sensor[2] < thresold)
		  {
			  PORTB|= (1<<PINB7);
			  PORTG|= (1<<PING3);
			  OCR1A=8000;
			  OCR1B=8000;			 //BACKWARD
			  
			  left_tx=OCR1A;
			  right_tx=OCR1B;
		  }
		  
		  	
		 
		  	USART_TransmitNumber(left_tx,0);
		  	USART_Transmitchar(' ',0);
		  	USART_TransmitNumber(right_tx,0);
		  	USART_Transmitchar(0x0D,0);
			_delay_ms(30);  
		  	
		 			
    }
}

ISR(USART0_RX_vect)
{
	distance=USART_Receive(0);
	
	if (distance == 'a')
	{
		j=0;
		Sensor[0] = 0;
	}
	else if(distance=='b')
	{
		j=1;
		Sensor[1] = 0;
		
	}
	else if (distance == 'c')
	{
		j = 2;
		Sensor[2] = 0;
	} 
	else
	{
		Sensor[j] = Sensor[j]*10 + (distance - '0');
	}
	
}

void right_wall_following()
{

	mearsured_distance=Sensor[2]; // right sensor
	
	error=limit-mearsured_distance;
	
	pid_calc=(kp * error+kd * (error-last_err));
	
	last_err=error;
	
	if(error>=-2 && error<=2)
	{
		OCR1A=8000;
		OCR1B=8000;
		
		left_tx=8000;
		right_tx=8000;
		
	}
	else
	{
		left_rpm=5000-pid_calc;
		right_rpm=5000+pid_calc;
		
		if(right_rpm>8000) right_rpm=8000;
		if(left_rpm>8000) left_rpm=8000;
		left_tx = left_rpm;
		right_tx = right_rpm;
		
		if(right_rpm<0)
		{
			PORTB|=(1<<PINB7);
			right_rpm=-right_rpm;

		}
		else{
			PORTB&=~(1<<PINB7);
		}
		
		if(left_rpm<0)
		{
			PORTG|=(1<<PING3);
			left_rpm=-left_rpm;
		}
		else{
			PORTG&=~(1<<PING3);
		}
		
		if(right_rpm>8000) right_rpm=8000;
		if(left_rpm>8000) left_rpm=8000;
		
		
		OCR1A=left_rpm;
		OCR1B=right_rpm;
		
	}
	
}

void left_wall_following()
{
	
	mearsured_distance=Sensor[1]; // right sensor
	
	error=limit-mearsured_distance;
	
	pid_calc=(kp * error+kd * (error-last_err));
	
	last_err=error;
		
	if(error>=-2 && error<=2)
	{
			
		OCR1A=8000;
		OCR1B=8000;
		
		left_tx=8000;
		right_tx=8000;
		
	}
	else
	{
		left_rpm=5000+pid_calc;
		right_rpm=5000-pid_calc;
		
		if(right_rpm>8000) right_rpm=8000;
		if(left_rpm>8000) left_rpm=8000;
		left_tx = left_rpm;
		right_tx = right_rpm;
		
		if(right_rpm<0)
		{
			PORTB|=(1<<PINB7);
			right_rpm=-right_rpm;

		}
		else{
			PORTB&=~(1<<PINB7);
		}
		
		if(left_rpm<0)
		{
			PORTG|=(1<<PING3);
			left_rpm=-left_rpm;
		}
		else{
			PORTG&=~(1<<PING3);
		}
		
		if(right_rpm>8000) right_rpm=8000;
		if(left_rpm>8000) left_rpm=8000;
		
		
		OCR1A=left_rpm;
		OCR1B=right_rpm;
		
	}
	
}
