#include "position_controller_1.h"
#include <xc.h>

void position_controller_init()
{
	//need to set up Timer for position controller
	//need to set up Digital output that gets toggled
	//200 Hertz 
	
	//timer for the 200Hz Timer -> Timer 4, priority 4
	//STEP 1: The ISR (located in main.c)
	//STEP 2: Dissable interrupts (this is called when disable interrupts is enabled)
	//STEP 3: setup peripheral 
	PR4 = 12499;					//for 200 kHz: (PR3 + 1)*N*12.5ns = 0.0002, if N = 32, then PR3 = 12499
	TMR4 = 0; 						//initialize count to 0 
	T4CONbits.TCKPS = 5; 			//set prescaler = 32 (see math work above)
	//T4CONbits.TGATE = 0;			//not gated input (default)
	IPC4bits.T4IP = 4;				//STEP 4: priority = 4 (in agreeance with definition in main) 
	IPC4bits.T4IS = 0;				//subpriority
	IFS0bits.T4IF = 0;				//STEP 5: clear interrupt flag
	IEC0bits.T4IE = 1;				//STEP 6: enable interrupt at CPU
	T4CONbits.ON = 1;				//Turn Timer4 ON
	//STEP 7: enable interrupts -> done in main 
	
	//Setting D7 to digital output --> RD7, pin 55
	TRISDbits.TRISD7 = 0;
	LATDbits.LATD7 = 0;
}

