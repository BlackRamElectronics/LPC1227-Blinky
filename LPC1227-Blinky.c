#include "LPC12xx.h"

volatile uint32_t msTicks = 0; // counter for 1ms SysTicks

/*----------------------------------------------------------------------------
  DEFINES
 *----------------------------------------------------------------------------*/

#define CLOCK_SETUP           1
#define MAINCLK_SETUP         1
#define SYSOSC_SETUP          1
#define SYSOSCCTRL_Val        0x00000000
#define SYSPLLCLKSEL_Val      0x00000001
#define SYSPLL_SETUP          1
#define SYSPLLCTRL_Val        0x00000041
#define MAINCLKSEL_Val        0x00000003
#define WDTOSC_SETUP          0
#define WDTOSCCTRL_Val        0x00000001
#define SYSAHBCLKDIV_Val      0x00000001
#define SSPCLKDIV_Val      	  0x00000001
#define UART0CLKDIV_Val       0x00000001
#define UART1CLKDIV_Val       0x00000001
#define RTCCLKDIV_Val      	  0x00000000
#define PMUCFG_SETUP          0
#define PMUCFG_Val            0x00000400
#define CLKOUTCLK_SETUP       0
#define CLKOUTCLKSEL_Val      0x00000000
#define CLKOUTCLKDIV_Val      0x000000FF

/*
//-------- <<< end of configuration section >>> ------------------------------
*/

/*----------------------------------------------------------------------------
  Check the register settings
 *----------------------------------------------------------------------------*/
#define CHECK_RANGE(val, min, max)                ((val < min) || (val > max))
#define CHECK_RSVD(val, mask)                     (val & mask)

/* Clock Configuration -------------------------------------------------------*/
#if (CHECK_RSVD((SYSOSCCTRL_Val),  ~0x00000003))
   #error "SYSOSCCTRL: Invalid values of reserved bits!"
#endif

#if (CHECK_RSVD((WDTOSCCTRL_Val),  ~0x000001FF))
   #error "WDTOSCCTRL: Invalid values of reserved bits!"
#endif

#if (CHECK_RANGE((SYSPLLCLKSEL_Val), 0, 2))
   #error "SYSPLLCLKSEL: Value out of range!"
#endif

#if (CHECK_RSVD((SYSPLLCTRL_Val),  ~0x000001FF))
   #error "SYSPLLCTRL: Invalid values of reserved bits!"
#endif

#if (CHECK_RSVD((MAINCLKSEL_Val),  ~0x00000003))
   #error "MAINCLKSEL: Invalid values of reserved bits!"
#endif

#if (CHECK_RANGE((SYSAHBCLKDIV_Val), 0, 255))
   #error "SYSAHBCLKDIV: Value out of range!"
#endif
 
/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/
#define __XTAL            (12000000UL)    /* Oscillator frequency             */
#define __SYS_OSC_CLK     (    __XTAL)    /* Main oscillator frequency        */
#define __IRC_OSC_CLK     (12000000UL)     /* Internal RC oscillator frequency */


#define __FREQSEL   ((WDTOSCCTRL_Val >> 5) & 0x0F)
#define __DIVSEL   (((WDTOSCCTRL_Val & 0x1F) << 1) + 2)

    #define __WDT_OSC_CLK        (1600000 / 2)	// WDTOSC_SETUP

    /* sys_pllclkin calculation */
    #define __SYS_PLLCLKIN           (__SYS_OSC_CLK)

    /* System PLL Setup         */
    //#define  __SYS_PLLCLKOUT         (__SYS_PLLCLKIN * ((SYSPLLCTRL_Val & 0x01F) + 1))
	#define  __SYS_PLLCLKOUT         (__SYS_PLLCLKIN * 2)

    /* main clock calculation */
    #define __MAIN_CLOCK             (__SYS_PLLCLKOUT)

    #define __SYSTEM_CLOCK             (__MAIN_CLOCK / SYSAHBCLKDIV_Val)

#define MAIN_OSC_FREQ 12000000
//#define MAIN_OSC_FREQ 4000000
void Init(uint32_t clock, uint32_t ahbdiv)
{
  uint32_t i;

  LPC_SYSCON->SYSMEMREMAP = 0x2;    /* remap to internal FLASH */
  
  // Sys Oscilator Enable
  LPC_SYSCON->SYSOSCCTRL = 0x00;
  
  // Power Up SYS Oscilator
  LPC_SYSCON->PDRUNCFG = 0;
  
  // Enable Internal RC oscilator
  //PDRUNCFG_bit.IRC_PD = 0;							!!!!!!!!!!!!
  //LPC_SYSCON->PDAWAKECFG &= ~(1 << 1);
  LPC_SYSCON->PDAWAKECFG = 0;
  
  // Select internal RC oscilator for Sys clock source
  LPC_SYSCON->MAINCLKUEN = 0;
  LPC_SYSCON->MAINCLKSEL = 0;
  LPC_SYSCON->MAINCLKUEN = 1;

  // Configure SYS PLL
  // Power Down SYS PLL
  //PDRUNCFG_bit.SYSPLL_PD = 1;
  LPC_SYSCON->PDAWAKECFG |= (1 << 7);
  
  // Select Sys Oscilator for SYS PLL source
  LPC_SYSCON->SYSPLLCLKUEN = 0;
  LPC_SYSCON->SYSPLLCLKSEL = 1;
  LPC_SYSCON->SYSPLLCLKUEN = 1;
  
  //Calc M
  uint32_t m = clock/MAIN_OSC_FREQ - 1;

  //assert(m<32);
  // Configure PLL frequency
  /*LPC_SYSCON->SYSPLLCTRL =  (m)    	// MSEL
             |  (0<<5) 				// PSEL = 1
             |  (0<<7) 				// DIRECT = 0
             |  (0<<8); 			// BYPASS=0*/
  //LPC_SYSCON->SYSPLLCTRL = 0x23;
  LPC_SYSCON->SYSPLLCTRL = 0x41;

  // Power Up PLL
  //PDRUNCFG_bit.SYSPLL_PD = 0;
  LPC_SYSCON->PDAWAKECFG &= ~(1 << 7);
  
  // Set Sys AHB Clock devider
  //SYSAHBCLKDIV_bit.DIV = ahbdiv;
  LPC_SYSCON->SYSAHBCLKDIV = ahbdiv;
  
  // Wain until PLL locks
  while(!(LPC_SYSCON->SYSPLLSTAT & 0x01));
  
  // Select Sys PLL Output for Sys clock source
  LPC_SYSCON->MAINCLKUEN = 0;
  LPC_SYSCON->MAINCLKSEL = 3;
  LPC_SYSCON->MAINCLKUEN = 1;
  
  
  return;

  // System Clock Setup
  // bit 0 default is crystal bypass, bit1 0=0~20Mhz crystal input, 1=15~50Mhz crystal input.
  LPC_SYSCON->SYSOSCCTRL = 0x00;
  // main system OSC run is cleared, bit 5 in PDRUNCFG register
  LPC_SYSCON->PDRUNCFG     &= ~(1 << 5);          /* Power-up System Osc      */
  LPC_SYSCON->SYSOSCCTRL    = SYSOSCCTRL_Val;
  // Wait 200us for OSC to be stablized, no status indication, dummy wait.
  for (i = 0; i < 200; i++) __NOP();

//SYSPLL_SETUP
  LPC_SYSCON->SYSPLLCLKSEL  = SYSPLLCLKSEL_Val;   // Select PLL Input        
  LPC_SYSCON->SYSPLLCLKUEN  = 0x01;               // Update Clock Source      
  LPC_SYSCON->SYSPLLCLKUEN  = 0x00;               // Toggle Update Register   
  LPC_SYSCON->SYSPLLCLKUEN  = 0x01;
  while (!(LPC_SYSCON->SYSPLLCLKUEN & 0x01));     // Wait Until Updated                                      
  LPC_SYSCON->SYSPLLCTRL    = SYSPLLCTRL_Val;	  // System PLL Setup         
  LPC_SYSCON->PDRUNCFG     &= ~(1 << 7);          // Power-up SYSPLL          
  while (!(LPC_SYSCON->SYSPLLSTAT & 0x01));	      // Wait Until PLL Locked    


  LPC_SYSCON->MAINCLKSEL    = MAINCLKSEL_Val;     // Select PLL Clock Output  
  LPC_SYSCON->MAINCLKUEN    = 0x01;               // Update MCLK Clock Source 
  LPC_SYSCON->MAINCLKUEN    = 0x00;               // Toggle Update Register   
  LPC_SYSCON->MAINCLKUEN    = 0x01;
  while (!(LPC_SYSCON->MAINCLKUEN & 0x01));       // Wait Until Updated       


/*#if (WDTOSC_SETUP)                                // Watchdog Oscillator Setup
  LPC_SYSCON->WDTOSCCTRL    = WDTOSCCTRL_Val;
  LPC_SYSCON->PDRUNCFG     &= ~(1 << 6);          // Power-up WDT Clock       
#endif*/

  LPC_SYSCON->SYSAHBCLKDIV  = SYSAHBCLKDIV_Val;
  LPC_SYSCON->SSPCLKDIV  = SSPCLKDIV_Val;
  LPC_SYSCON->UART0CLKDIV= UART0CLKDIV_Val;
  LPC_SYSCON->UART1CLKDIV= UART1CLKDIV_Val;
  LPC_SYSCON->RTCCLKDIV  = RTCCLKDIV_Val;

/*#if (PMUCFG_SETUP)                                // PMU CFG Setup
  if ( (LPC_PMU->SYSCFG&(0x1<<10)) != ((PMUCFG_Val&(0x1<<10))) )
  {
	LPC_PMU->SYSCFG &= ~(0x1<<10);
	LPC_PMU->SYSCFG |= PMUCFG_Val&(0x1<<10); 
  }

  // Only update clock source if it hasn't been set previously.
  //Thus when waking from Deep Power Down settings are not overwritten
  if ( (LPC_PMU->SYSCFG&(0xF<<11)) != ((PMUCFG_Val&(0xF<<11))) )
  {
	LPC_PMU->SYSCFG &= ~(0xF<<11);
	LPC_PMU->SYSCFG |= PMUCFG_Val&(0xF<<11); 
  }
  // The RTC clock source must be selected before the RTC is enabled
  LPC_SYSCON->SYSAHBCLKCTRL |= (1<<19);
#endif*/

  /* System clock to the IOCON needs to be enabled or
  most of the I/O related peripherals won't work. */
  LPC_SYSCON->SYSAHBCLKCTRL |= (1<<16);
}


//====================================================================================
void main()
{
	uint32_t timer_mark;
	
	//Init(132000000, 1);

	// Enable GPIO blocks and IOCON
	LPC_SYSCON->SYSAHBCLKCTRL |= 0xE001001F;
	
	// Configure GPIO 0.0
	//LPC_IOCON->PIO0_0 = 0;
	//LPC_IOCON->PIO0_0 &= ~(1 << 4);
	LPC_IOCON->PIO0_0 = 0x00;
	LPC_GPIO0->DIR |= 1 << 0;
	LPC_GPIO0->CLR = 1 << 0;
	//for(;;);
	
	//LPC_GPIO0->DIR = 1 << 7;
	//LPC_GPIO0->CLR = 1 << 7;
	
	// Enable GPIO Clock ( powers the GPIO peripheral )
	//LPC_SYSCON->SYSAHBCLKCTRL |= (1<<6);
	//LPC_SYSCON->SYSAHBCLKCTRL |= 0xE001001FUL;

	// Select GPIO Mode and disable analog mode, refer to User Manual - UM10524
	LPC_IOCON->PIO1_4 = (1 << 7) | (1 << 9);
	//LPC_IOCON->PIO1_4 |= (1 << 9);
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
