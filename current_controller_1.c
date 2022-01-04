#include "current_controller_1.h"
#include <xc.h>

void current_controller_init()
{
	//timer for the 5kHz Timer -> Timer 2
	//STEP 1: The ISR (located in main.c)
	//STEP 2: Dissable interrupts (this is called when disable interrupts is enabled)
	//STEP 3: setup peripheral 
	PR2 = 15999;					//for 5 kHz: (PR1 + 1)*N*12.5ns = 0.0002, if N = 1, then PR1 = 15999
	TMR2 = 0; 						//initialize count to 0 
	T2CONbits.TCKPS = 0; 			//set prescaler = 1 (see math work above)
	T2CONbits.TGATE = 0;			//not gated input (default)
	IPC2bits.T2IP = 5;				//STEP 4: priority = 5 (in agreeance with definition in main) 
	IPC2bits.T2IS = 0;				//subpriority
	IFS0bits.T2IF = 0;				//STEP 5: clear interrupt flag
	IEC0bits.T2IE = 1;				//STEP 6: enable interrupt at CPU
	T2CONbits.ON = 1;
	//STEP 7: enable interrupts -> done in main 
	
	//----------------------------------------------------------------
	//timer for the 20kHz PWM signal -> Timer 3
	//for 20kHz: (PR3+1)*N*12.5ns = 0.00005, if N = 1, then PR3 = 3999
	PR3 = 3999;
	TMR3 = 0;
	T3CONbits.TCKPS = 0;
	T3CONbits.TGATE = 0;			//not gated input (default)
	//doesnt need priority because not for ISR
	//doesn't need subpriority
	//dont need to clear the interrupt flag
	
	//Output Compare for PWM -> OC5, pin 52, D4
	//duty cycle = OC5R(PR3+1)*100% (page 134)
	OC5CONbits.OCM = 0b110; 	//PWM mode without fault pin (page 137)
	OC5CONbits.OCTSEL = 1; 		//setting up the use with Timer 3
	OC5RS = 0;
	OC5R = 0;					//initialize before turning on OC5
	T3CONbits.ON = 1;			//turn on the timer
	OC5CONbits.ON = 1;			//turn on OC5
	//----------------------------------------------------------------
	
	//Digital Output to control the motor direction -> RB7, pin 18, B7
	TRISBbits.TRISB7 = 0;					//B7 configured as output
	LATBbits.LATB7 = 0;						//configure output as low
}
	