#include "NU32.h"		//config bits, constants, funcs for startup and UART
#include "encoder_1.h"
#include "ADC_1.h"
#include "utilities_1.h"
#include "current_controller_1.h"
#include "position_controller_1.h"
#include <stdio.h>

#define BUF_SIZE 200
#define ARRAY_SIZE 100
#define U_MAX 100
#define U_MIN -100

static volatile int duty_cycle; 
static volatile int mode; 
static volatile int plotting_index;
static volatile int ref_current;
static volatile int current_counter;

//PI current control variables
static volatile float Kp_current; 
static volatile float Ki_current;
static volatile float e;
static volatile float current_sensor_ma;	
static volatile float e_current;
static volatile float eint;
static volatile float Ti;
static volatile float Tp;
static volatile float u;
static volatile float u_dc;

//PID position controller variables
static volatile float Kp_position; 
static volatile float Ki_position;
static volatile float Kd_position;
static volatile float eint_pos;
static volatile float ed_pos;
static volatile float e_pos;
static volatile float u_pos;
static volatile float u_dc_pos;
static volatile float Ti_pos;
static volatile float Tp_pos;
static volatile float Td_pos;
static volatile float encoder_reading;
static volatile float encoder_reading_prev;
static volatile float u_ref;

//arrays to hold points for graphing in MATLAB
static volatile int REF_CURRENT_ARRAY[ARRAY_SIZE];
static volatile int ACTUAL_CURRENT_ARRAY[ARRAY_SIZE];



//Interrupt Service Routines ----------------------------------------------------------------------
void __ISR(_Timer_4_Vector, IPL4SOFT) PositionControllerISR(void){
	
	switch(mode) //1 - idle, 2 - pwm, 3 - itest, 4 - hold, 5 - track
	{
		case 1:		//IDLE
		{
			break;
		}
		case 2:		//PWM				
		{
			break;
		}
		case 3:		//ITEST
		{
			break;
		}
		case 4:		//HOLD
		{
			//steps:
			//read encoder (in degrees)
			//compare against desired
			//calculate a reference current using PID 
			
			encoder_reading = ((encoder_counts()-32768)*360)/(28*4);
			
			//PID controller
			e_pos = desired_angle - encoder_reading;
			eint_pos = eint_pos + e_pos;
			ed_pos = encoder_reading - encoder_reading_prev;
			
			Tp_pos = e_pos*Kp_pos;
			Ti_pos = eint_pos*Ki_pos;
			Td_pos = ed_pos*Kd_pos;
			
			u_pos = Tp_pos+Ti_pos+Td_pos;
			
			//U max for position???
			
			encoder_reading_prev = e_pos;
			break;
		}
		case 5:		//TRACK
		{
			break;
		}
		
	}
	IFS0bits.T4IF = 0;			//clear interrupt flag
}

void __ISR(_TIMER_2_VECTOR, IPL5SOFT) CurrentControllerISR(void){

	
	switch (mode)
	{
		case 1:	//IDLE (Puts H-bridge in brake mode -> 0% duty cycle)
		{
			OC5RS = 0;												//duty cycle = 0
			break;
		}
		case 2:	//PWM 												//OCxR = (duty_cycle/(PRy+1))/100
		{
			if (duty_cycle > 0)										//forward
				{
					if (duty_cycle <= 100)							//range: [0,100]
					{
						LATBbits.LATB7 = 0;							//binary low = forward
						OC5RS = ((duty_cycle*(3999+1))/100); 		//PR2 = 3999
					}			
					else 											// >100
					{
						LATBbits.LATB7 = 0;
						OC5RS = ((100*(3999+1))/100); 				//PR2 = 3999 
					}
				}		
			else // < 0 			
				{
					if (duty_cycle >= -100)							//range: [0,-100]
					{
						LATBbits.LATB7 = 1;							//binary high = reverse
						OC5RS = ((duty_cycle*(3999+1))/100)*(-1); 	//PR2 = 3999
					}
					else											// <-100 
					{
						LATBbits.LATB7 = 1;	
						OC5RS = ((100*(3999+1))/100); 				//PR2 = 3999
					}	 							
				}

				break;
		}
		case 3: //ITEST
		{
			
			if (current_counter >= 0 && current_counter < 25)				//0 - 24
				{
					ref_current = 200;
				}					
			if (current_counter >= 25 && current_counter < 50)				//25 - 49
				{
					ref_current = -200;
				}			
			if (current_counter >= 50 && current_counter < 75)				//50 - 74	
				{
					ref_current = 200; 
				}			
			if (current_counter >= 75 && current_counter < 99)				//75 - 98
				{
					ref_current = -200; 
				}				
			
			//PI controller 
				//1) readings the current sensor
				//2) compares i to the square wave reference 
				//3) calculates new PWM duty cycle and motor direction bit (Duty cycle is duty_cycle and motor direction bit = OC5RS)
				//4) actual current data and reference are saved in arrays for later plotting (since counter 0 - 99 there will be 100 counts in array)
				
				//1) 
				current_sensor_ma = (3.00*ADC_sample_convert()-1491.0)-55;
				
				//2) -> based on PID controller code from Assignment 23	(note, there is no dT because we have counter going within ISR)
				e = ref_current - current_sensor_ma;
				
				//integral term
				eint = eint + e;
				Ti = Ki_current*eint;
				
				//proportional 
				Tp = Kp_current*e;
				
				//actuator command 
				u = Tp+Ti;
				
				//3) new PWM duty cycle and motor direction bit
				//if u < 0 -> motor in reverse, if u > 0 -> motor in forward, if u = 0 -> nothing
				if (u>U_MAX)			//checking against U max
				{
					u = 100;
				}
				if (u<U_MIN)
				{
					u = -100;
				}
				
				if(u<0)			//time to output to digital pin B7
				{
					LATBbits.LATB7 = 1;				//reverse
					u_dc = -u;
					OC5RS = ((u_dc)/(100))*(3999+1);
				}
				if(u>0) 
				{
					LATBbits.LATB7 = 0;				//forward
					u_dc = u;
					OC5RS = ((u_dc)/(100))*(3999+1);
				}
				

				//store in arrays for plotting 
				REF_CURRENT_ARRAY[current_counter] = ref_current;
				ACTUAL_CURRENT_ARRAY[current_counter] = current_sensor_ma;
				
				current_counter = current_counter+1;
				
				if (current_counter == 100)	//indicated we are done and gonna plot in MATLAB
					{
						mode = 1;
						//e = 0;				//reset all the varibales for completely new cycle next time around
						//eint = 0; 			//upon second thought, might not be able to do this because we might need these values for the PID position controller
						//Tp = 0;
						//Ti = 0;
						//u = 0;
						//u_dc = 0;
						current_counter = 0;
					}													
				
				break;
		}
		case 4: //HOLD
		{
			u_reference = u_pos;	//from PID of position controller
			
			current_sensor_ma = (3.00*ADC_sample_convert()-1491.0)-55;
			e = ref_current - current_sensor_ma;
			
			//integral term
			eint = eint + e;
			Ti = Ki_current*eint;
			
			//proportional 
			Tp = Kp_current*e;
			
			//actuator command 
			u = Tp+Ti;
			
			//3) new PWM duty cycle and motor direction bit
			//if u < 0 -> motor in reverse, if u > 0 -> motor in forward, if u = 0 -> nothing
			if (u>U_MAX)			//checking against U max
			{
				u = 100;
			}
			if (u<U_MIN)
			{
				u = -100;
			}
			
			if(u<0)			//time to output to digital pin B7
			{
				LATBbits.LATB7 = 1;				//reverse
				u_dc = -u;
				OC5RS = ((u_dc)/(100))*(3999+1);
			}
			if(u>0) 
			{
				LATBbits.LATB7 = 0;				//forward
				u_dc = u;
				OC5RS = ((u_dc)/(100))*(3999+1);
			}
			
			
			break;
		}
		case 5: //TRACK
		{
			break;
		}
	}
	
	IFS0bits.T2IF = 0; 				//clear the interrupt flag
}

//main function -----------------------------------------------------------------------------------
int main()
{
	char buffer[BUF_SIZE];
	NU32_Startup();
	NU32_LED1 = 1;
	NU32_LED2 = 1;
	
	__builtin_disable_interrupts();
	
	//initializing modules and peripherals--------------------------------
	encoder_init();					//initilize encoder
	ADC_init();						//initilize ADC 
	current_controller_init();		//initilize curreent sensor	
	position_controller_init();				
	mode = 1;						//start in IDLE mode
	
	//PI for current controller initilization
	e = 0;							
	eint = 0; 						 
	Tp = 0;							
	Ti = 0;
	u = 0;
	u_dc = 0;
	current_counter = 0;
	
	//PID for position controller initilization
	e_pos = 0;
	eint_pos = 0;
	ed_pos = 0;
	u_pos = 0;
	u_dc_pos = 0;
	Ti_pos = 0;
	Tp_pos = 0;
	Td_pos = 0;
	
	__builtin_enable_interrupts();
	
	while(1)
	{
		NU32_ReadUART3(buffer,BUF_SIZE);
		NU32_LED2 = 1;
		switch(buffer[0]) {
						case 'a':
						{
							ADC_sample_convert();
							int ADC_counts = ADC_sample_convert();
							sprintf(buffer, "%d\r\n", ADC_counts);
							NU32_WriteUART3(buffer);
							break;
						}
						case 'b':
						{
							float current_ma = (3.00*ADC_sample_convert()-1491.0)-55;
							sprintf(buffer, "%f\r\n", current_ma);
							NU32_WriteUART3(buffer);
							break;
						}
						case 'c':
						{
							sprintf(buffer,"%d\r\n",encoder_counts());
							NU32_WriteUART3(buffer); 						//send encoder count to client
							break;
						}
						case 'd':
						{
							float encoder_degrees = ((encoder_counts()-32768)*360)/(28*4);		//changed from double -> float
							//28 CPR with 4x resolution and quad encoding
							sprintf(buffer,"%f\r\n",encoder_degrees);
							NU32_WriteUART3(buffer);
							break;
						}
						case 'e':
						{
						    encoder_reset();		//might want to consider printing after this command if debugging
							break;
						}
						case 'f':
						{
							int o = 0;
							NU32_ReadUART3(buffer, BUF_SIZE);		//read input from user
							sscanf(buffer, "%d", &o);		//take input from user for duty cycle value
							duty_cycle = o;
							
							mode = 2;
							
							break;
						}
						case 'g':				//Set current gains
						{
							float i = 0;
							float j = 0;
							//reading from UART Kp and Ki
							NU32_ReadUART3(buffer,BUF_SIZE);
							sscanf(buffer, "%f", &i);
							NU32_ReadUART3(buffer,BUF_SIZE);
							sscanf(buffer, "%f", &j);
							
							Kp_current = i;
							Ki_current = j;
							
							sprintf(buffer,"%f\r\n",i);
							NU32_WriteUART3(buffer);
							sprintf(buffer,"%f\r\n",j);
							NU32_WriteUART3(buffer);

							break;
							
						}
						case 'h':
						{
							//sending Kp and Ki over UART
							float k = Kp_current;
							float m = Ki_current; 
							
							sprintf(buffer,"%f\r\n", k);
							NU32_WriteUART3(buffer);
							
							sprintf(buffer,"%f\r\n", m);
							NU32_WriteUART3(buffer);
							
							break;
						}
						case 'i':
						{
							float z = 0;
							float x = 0;
							float y = 0;
							
							//reading from UART Kp,Ki,Kd
							NU32_ReadUART3(buffer,BUF_SIZE);
							sscanf(buffer, "%f", &z);
							NU32_ReadUART3(buffer,BUF_SIZE);
							sscanf(buffer, "%f", &x);
							NU32_ReadUART3(buffer,BUF_SIZE);
							sscanf(buffer, "%f", &y);
							
							Kp_position = z;
							Ki_position = x;
							Kd_position = xy
							
							sprintf(buffer,"%f\r\n",z);
							NU32_WriteUART3(buffer);
							sprintf(buffer,"%f\r\n",x);
							NU32_WriteUART3(buffer);
							sprintf(buffer,"%f\r\n",y);
							NU32_WriteUART3(buffer);
							
							break;
						}
						case 'j':
						{
							float t = Kp_position;
							float r = Ki_position;
							float v = Kd_position;
							
							sprintf(buffer,"%f\r\n",t);
							NU32_WriteUART3(buffer);
							
							sprintf(buffer,"%f\r\n",r);
							NU32_WriteUART3(buffer);
							
							sprintf(buffer,"%f\r\n",v);
							NU32_WriteUART3(buffer);
							
							break;
						}
						case 'k':			//Test current gains
						{
							mode = 3; //ITEST mode
								
							while(mode == 3)				//waiting for end of ISR to turn mode back to 1 before proceeding
							{
								;
							}
							
							int a;
							int c;
							int N = 100;
							plotting_index = 0;
							
							//matlab function first needs number of samples = 100
							sprintf(buffer,"%d\r\n", N);
							NU32_WriteUART3(buffer);
							
							while(plotting_index < 100)
							{
								c = REF_CURRENT_ARRAY[plotting_index];
								a = ACTUAL_CURRENT_ARRAY[plotting_index];
								
								sprintf(buffer,"%d %d\r\n", c, a);
								NU32_WriteUART3(buffer);
								
								plotting_index = plotting_index+1;
							}
							
							break;
							
						}
						case 'l':
						{
							float desired_angle = 0;		//degrees 
							
							NU32_ReadUART3(buffer,BUF_SIZE);
							sscanf(buffer, "%f", &desired_angle);
							
							mode = 4; 						//hold
							
						}
						case 'm'	//load step trajectory
						{
							break;
						}
						case 'n'	//load cubic trajectory
						{
							break;
						}
						case 'o'	//execute trajectory
						{
							break;
						}
						case 'r':
						{
							switch(mode) 	
							{
								
								case 1: //IDLE
								{
									sprintf(buffer,"%s\r\n", "IDLE");
									break;
								}
							
								case 2: //PWM
								{
									sprintf(buffer,"%s\r\n", "PWM");
									break;
								}
							
								case 3:	//ITEST
								{
									sprintf(buffer,"%s\r\n", "ITEST");
									break;
								}
							
								case 4: //HOLD
								{
									sprintf(buffer,"%s\r\n", "HOLD");
									break;
								}
							
								case 5: //TRACK
								{
									sprintf(buffer,"%s\r\n", "TRACK");
									break;
								}
							
							}
							
							NU32_WriteUART3(buffer);
							break;
							
						}
						case 'q':
						{
							mode = 1;									//set to IDLE before quitting
							sprintf(buffer,"%s\r\n","IDLE");			//write IDLE to client
							NU32_WriteUART3(buffer);					//write via UART
							break;
						}
						case 'p':										//unpower the motor
						{
							mode = 1;									//set mode to idle
							break;
						}
						default:
						{
							NU32_LED2 = 0; 								//turn on LED2 to indicate an error
							break;
						}
		}
	}
	return 0;
}