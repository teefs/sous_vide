#include <stdint.h>
#include "msp.h"
#include "lcd.h"


/**
 * main.c
 */
void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

	initLCD();

    SysTick->LOAD = 0x00E4E1C0 - 1; // 3MHz clk = 5 second countdown
    SysTick->VAL = BIT0;
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;

    SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;    // Enable sleep on exit from ISR
    __DSB();

    __enable_irq();
    __wfi();
}

void SysTick_Handler(void)
{
    P1->OUT ^= BIT0;                        // Toggle P1.0 LED
}
