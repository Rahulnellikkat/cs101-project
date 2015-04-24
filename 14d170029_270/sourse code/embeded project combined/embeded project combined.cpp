/*
 * moving_obstacle_avoiding.cpp
 *
 * Created: 4/8/2015 6:58:14 PM
 *  Author: user
  */ 


#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> //included to support power function
#include "lcd.h"


unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char sharp, distance, adc_reading;
unsigned int value;
unsigned int value1;
unsigned int value2;
int velocity_of_object;
unsigned int time_to_reach ;
float BATT_Voltage, BATT_V;

unsigned char flag = 0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;

//buzzer
//Function to initialize Buzzer
void buzzer_pin_config (void)
{
	DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

void buzzer_on (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}

void buzzer_off (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}

//buzzer

void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}void adc_pin_config (void)
{
 DDRF = 0x00; //set PORTF direction as input
 PORTF = 0x00; //set PORTF pins floating
 DDRK = 0x00; //set PORTK direction as input
 PORTK = 0x00; //set PORTK pins floating
}
//Function to configure ports to enable robot's motion
void motion_pin_config(void)
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

void port_init()
{
	lcd_port_config();
	adc_pin_config();
	motion_pin_config();
	//buzzer
	buzzer_pin_config();
}

void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}
void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 			// removing upper nibbel as it is not needed
	PortARestore = PORTA; 			// reading the PORTA's original status
	PortARestore &= 0xF0; 			// setting lower direction nibbel to 0
	PortARestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTA status
	PORTA = PortARestore; 			// setting the command to the port
}
void forward (void) //both wheels forward
{
	motion_set(0x06);
}

void back (void) //both wheels backward
{
	motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
	motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
	motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
	motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
	motion_set(0x08);
}

void stop (void)
{
	motion_set(0x00);
}
//adc_stuff

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}
unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}


void print_sensor(char row, char coloumn,unsigned char channel)
{
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}

void init_devices (void)
{
	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	timer5_init();
	
	sei(); //Enables the global interrupts
}
void avioiding_object_turning(void)
{
	 velocity(250,250);
	 soft_right(); //Left wheel forward, Right wheel is stationary
	 _delay_ms(1225);
	 forward();
	 _delay_ms(220);
	 soft_left();
	 _delay_ms(1094);
	 stop();
	 sharp=ADC_Conversion(11);
	 value=Sharp_GP2D12_estimation(sharp);
	 if (value >500)
	 {
	 velocity(250,250);
	 forward();
	 }
	 else
	 {
		 while (value > 150)
		 {
			 forward();
		 }
		 avioiding_object_turning();
		 
	 }	 
	 
}
void avioiding_object_turningback ()
{   
	forward();
	_delay_ms(1000);
	
	soft_left();
	_delay_ms(1000);
	forward();
	_delay_ms(300);
	soft_right();
	_delay_ms(1000);
	forward();
}
void velocity_of_object_finder (unsigned int i)
{
	stop();
	_delay_ms(10);
	sharp = ADC_Conversion(11);
	value1 = Sharp_GP2D12_estimation(sharp);
	_delay_ms(500);
	sharp = ADC_Conversion(11);
	value2 = Sharp_GP2D12_estimation(sharp);
	velocity_of_object =(value2 - value1)*2;
	i = 0;
}
void linefollowerfunction(void)
{

	Left_white_line = ADC_Conversion(3);				//Getting data of Left WL Sensor
	Center_white_line = ADC_Conversion(2);				//Getting data of Center WL Sensor
	Right_white_line = ADC_Conversion(1);				//Getting data of Right WL Sensor

	flag=0;
	if(Center_white_line < 0x10)
	{
		flag=1;
		forward();
		velocity(100,100);
	}

	if((Left_white_line > 0x10) && (flag==0))
	{
		flag=1;
		forward();
		velocity(130,50);
	}

	if((Right_white_line > 0x10) && (flag==0))
	{
		flag=1;
		forward();
		velocity(50,130);
	}

	if(Center_white_line > 0x10 && Left_white_line>0x10 && Right_white_line>0x10)
	{
		forward();
		velocity(0,0);
	}

}

void aranging_bot(void)
{
	velocity(150,150);
	soft_left(); //Left wheel backward, right wheel stationary
	_delay_ms(373);
	soft_right();
	_delay_ms(461);
	stop();
	back();
	_delay_ms(422);
	stop();
	_delay_ms(10);
}
void right_angle(void)
{   velocity(100,100);
	back();
	_delay_ms(610);
	
	soft_right(); //Left wheel forward, Right wheel is stationary
	_delay_ms(3290);
	
	back();
	_delay_ms(1780);

	stop();
}
unsigned int obstacle_coming_towards_bot(void)
{
	while(1)
	{ 
		back();
		if (ADC_Conversion(8) > 120)
		{
		soft_left_2(); //Left wheel backward, right wheel stationary
		_delay_ms(1000);
		
		back();
		_delay_ms(750);

		stop();
		while(1)
		{           //coming back
			sharp = ADC_Conversion(11);
			value = Sharp_GP2D12_estimation(sharp);
			if(value < 700) // object
			{
				while(1)
				{
					_delay_ms(3000);
					sharp = ADC_Conversion(11);
					value = Sharp_GP2D12_estimation(sharp);
					if(value >= 790)// taking error in to account
					{
						forward ();
						_delay_ms(750);
						soft_right() ;
						_delay_ms (1000);
						forward();
						return 1;
					}
					
					
				}
			}
			
			
		}
	}
 } 
}
unsigned int obstacle_going_away__from_bot(void)
{
	while(1)
	{
		sharp = ADC_Conversion(11);
		value = Sharp_GP2D12_estimation(sharp);
		if (value > 150)
		{
			linefollowerfunction();
		}
		else
		{
			avioiding_object_turning();
			_delay_ms(1000); /*some time it don't sence the obstacle in left so what we do is go forward for 1 s then we webelive 
			                 that obstacle is in front of adc_conver(4) the  it understand when it is avoided*/ 
			while(1)
			{
				if(ADC_Conversion(4) > 140)
				{
					velocity(250,250);
					avioiding_object_turningback ();
					
					return 1 ;
				}
				
			}
		}
	}
}
 unsigned int stationary_obstaclle_avoiding(void)
 {
	 unsigned int i=100;
	 while (1)
	 {
		 //sharp = ADC_Conversion(11);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
		 //value = Sharp_GP2D12_estimation(sharp);
		 //velocity(100,100);
		 //forward();
		 if(ADC_Conversion(6) < 90)
		 {
			 velocity(100,100);
			 right_angle();// turing the object by an angle 90* and and bringing the IR proxy senser at the right point
			 while(1)
			 {
				 if(ADC_Conversion(4) > 105)
				 {
					 aranging_bot();
				 }
				 else
				 break;
			 }
			 adc_reading = ADC_Conversion(4);
			 while (i)
			 {
				 i--;
				 //starting  the obstacle avoiding main body
				 forward();
				 _delay_ms(10);
				 stop();
				 if (ADC_Conversion(4) < adc_reading - 20)  //going close to the obstacle
				 {
					 velocity(80,58);
				 }
				 else if (ADC_Conversion(4) < adc_reading)
				 {
					 velocity(80,70);
				 }
				 else if (ADC_Conversion(4) > 120)  // no obstacle before sensor number 4 (proximity sensor on left)
				 {   velocity(100,100);
					 forward();
					 _delay_ms(400);
					 velocity(50,140);
					 while (1)
					 {
						 if (ADC_Conversion(4) > 80)
						 {
							 forward();
							 _delay_ms(10);
						 }
						 else break;
						 
					 }
					 
				 }
				 else if (ADC_Conversion(4) > adc_reading) // going away from the object taking error in to acount
				 {
					 velocity(58,80);
				 }
				 else
				 {
					 velocity(80,80);
				 }
			 }
			 Left_white_line = ADC_Conversion(3);				//Getting data of Left WL Sensor
			 Center_white_line = ADC_Conversion(2);				//Getting data of Center WL Sensor
			 Right_white_line = ADC_Conversion(1);				//Getting data of Right WL Sensor
			 if(Center_white_line < 0x10)
			 return 1;
			 else
			 if(Left_white_line < 0x10)
			 return 1;
			 else
			 if(Right_white_line < 0x10)
			 return 1;
			 else
			 //starting  the obstacle avoiding main body
			 forward();
			 stop();
			 if (ADC_Conversion(4) < adc_reading - 20)  //going close to the obstacle
			 {
				 velocity(80,58);
			 }
			 else if (ADC_Conversion(4) < adc_reading)
			 {
				 velocity(80,70);
			 }
			 else if (ADC_Conversion(4) > 120)  // no obstacle before sensor number 4 (proximity sensor on left)
			 {   velocity(100,100);
				 forward();
				 _delay_ms(400);
				 velocity(50,140);
				 while (1)
				 {
					 if (ADC_Conversion(4) > 80)
					 {
						 forward();
						 _delay_ms(10);
					 }
					 else break;
					 
				 }
				 
			 }
			 else if (ADC_Conversion(4) > adc_reading) // going away from the object taking error in to acount
			 {
				 velocity(58,80);
			 }
			 else
			 {
				 velocity(80,80);
			 }
			 
			 
		 }
		 else
		 {
			 linefollowerfunction();
		 }
	 }
	 
 }
void back_bone_function(void) //back  bone of bot
{
	while(1)
	{
		linefollowerfunction();
		 sharp = ADC_Conversion(11);
		 value = Sharp_GP2D12_estimation(sharp);
		 if (value <= 300)
		 {
			 velocity_of_object_finder(1);
		 }
		 if (velocity_of_object < 10) //taking error in to account //object coming towards bot
		 {
			if (obstacle_coming_towards_bot() == 1)
			{
				back_bone_function();//obstacle avoided//going back to the back bone function
			}
		 }
		 else if ( velocity_of_object > 10)
		 {
			if (obstacle_going_away__from_bot() ==1)
			{
				back_bone_function();//obstacle avoided//going back to the back bone function
			}
			 
		 }
		 else
		 {
			 if (stationary_obstaclle_avoiding() == 1)
			 {
				 back_bone_function();//obstacle avoided//going back to the back bone function
			 }
		 }
		 }	 
	
}
int main(void)
{
  init_devices();
  back_bone_function();
}	
