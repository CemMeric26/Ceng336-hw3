#include "pragmas.h"
#include <xc.h>


void transmitCharAndHello(char chr)
{
    /**/ */
}

void transmitData()
{
    /* */
}


void __interrupt (high_priority) highPriorityISR (void) {
		//
}

void init_int() {
    
    /* configure I/O ports */
   
	/* configure USART transmitter/receiver */
	// TXSTA1: Configure for Asynchronous mode, High-speed baud rate, 8-bit transmission, Transmitter initially disabled.

    // RCSTA1: Configure for Serial Port Enable, Continuous Receive, 8-bit reception.
  
    /* configure the interrupts */
 
	
}

void init_adc(){
  // Set ADC Inputs

  // Configure ADC
  

}

void main(void) {
    init_adc();
    init_int();
    while(1) {
        // Get ADC Sample
        GODONE = 1; // Start ADC conversion
       
    }
    return;
    
}
