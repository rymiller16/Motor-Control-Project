#include "ADC_1.h"
#include <xc.h>

#define SAMPLE_TIME 10

	//per page 147 -> need minimum 65ns spec thus we need to choose k=3, Tad = 75ns which gived ADCS = 2
	//configure AN7 as the input pin per pin configuration in first steps of project 
	//choosing auto sampling
	//Based on example 10.1

void ADC_init() {
	
    AD1PCFGbits.PCFG0 = 0;    //CONFIGURE AN0 AS THE ANALOG INPUT PIN = Pin B0 on PIC
    AD1CON3bits.ADCS = 2;     //conversion clock select bit, TAD = 75ns, Tpb = 12.5ns
	AD1CON1 = 0x00E0; 		  //SSRC bit = 111, int. counter ends sampling and starts conv
    AD1CON1bits.ASAM = 1;     //autostart sampling
    AD1CON1bits.ADON = 1;     //turn on ADC (should be last step in initilization)*/
	
}

	//based on the code from example 10.1
	//sample and convert the value of the given adc pin, the pin should be configured as an analog
	//input in AD1PCFG (it is above)

unsigned int ADC_sample_convert() {
		
	
	unsigned int elapsed = 0, finish_time = 0;
	AD1CHSbits.CH0SA = 0;						//chosen pin to MUXA for sampling

	AD1CON1bits.SAMP = 1;						//start sampling
	elapsed = _CP0_GET_COUNT();
	finish_time = elapsed+SAMPLE_TIME;
	
	while(_CP0_GET_COUNT()<finish_time)
	{
		;										//sample for more than 250 ns
	}
	
	AD1CON1bits.SAMP = 0;						//stop sampling and start converting
	
	while(!AD1CON1bits.DONE)
	{
		;										//wait for conversion process to finish
	}

	int i = 0;
	float adder = 0;
	float result = 0;
	
	while(i<=31)								//average 32 samples and then output for more consistency
	{
		adder += ADC1BUF0;
		i++;
	}
	result = (adder)/32.0;
	return result;								//read the buffer with the result
}