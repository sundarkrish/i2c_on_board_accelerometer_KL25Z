#include <stdint.h>
#include "MKL25Z4.h"
#include "KL25Z_delay.h"



// Creates a delay of 1ms x count
// e.g. count = 500, 1ms x 500 = 500ms delay

void delay(uint16_t count)
{
	SysTick->LOAD = 48000 - 1; // 1ms delay initial SYST_RVR value
    SysTick->CTRL = 0x5; // Start SysTick Timer,Processor clock source,No interrupt

	while(count != 0)
	{
		while((SysTick->CTRL & (1 << 16) ) == 0);
		count--;
	}

}
