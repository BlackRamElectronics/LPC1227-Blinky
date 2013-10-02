#include "LPC12xx.h"

volatile uint32_t msTicks = 0; // counter for 1ms SysTicks

//====================================================================================
void main()
{
	uint32_t timer_mark;

	// Enable GPIO blocks and IOCON
	LPC_SYSCON->SYSAHBCLKCTRL |= 0xE001001F;

	// Select GPIO Mode and disable analog mode, refer to User Manual for more detail
	LPC_IOCON->PIO1_4 = (1 << 7) | (1 << 9);
	LPC_IOCON->PIO1_5 = (1 << 7) | (1 << 9);
	
	// Set the pin direction, set high for an output
	LPC_GPIO1->DIR |= 1 << 4;
	LPC_GPIO1->DIR |= 1 << 5;
	
	LPC_GPIO1->CLR = 1 << 4;
	LPC_GPIO1->SET = 1 << 5;

	// Init SysTick
	SysTick_Config(SystemCoreClock / 1000);				// Generate interrupt every 1 ms
	
	for(;;)
	{
		timer_mark = msTicks;					// Take timer snapshot 
		while(msTicks < (timer_mark + 100));	// Wait until 100ms has passed
		LPC_GPIO1->NOT = 1 << 4;
		LPC_GPIO1->NOT = 1 << 5;
	}
}


//====================================================================================
void SysTick_Handler(void)
{
	msTicks++;
}
