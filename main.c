#include <stdint.h>
#include <stdbool.h>
#include "msp.h"
#include "lcd.h"

const int outLower = 128, outUpper = 1152;
int output;

float sampleToTemp (void);

void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

	// Configure peripheral devices and IO.

	/* Configure clocks.
	 * LFTXCLK = 32.768kHz.
	 *      Clocks LCD_F.
	 * ACLK = 512Hz. (Default ACLK source is LFTXCLK, then divided it by 64.)
	 *      Clocks TIMER_A0.
	 * MCLK = 3MHz. (Default MCLK source is DCOCLK set to 3MHz.)
	 *      Clocks ADC14, ARM processor, and on-processor peripherals.
	 */
    CS->KEY = CS_KEY_VAL;
    CS->CTL1 = CS_CTL1_DIVA__64;

    /* ADC14
     * Configured to repeatedly sample on pin 5.4.
     * V+ = Vref (Buffered) = 1.45V
     * V- = Vcc = Gnd
     */
    REF_A->CTL0 |= REF_A_CTL0_ON | REF_A_CTL0_VSEL_1;
    P5->SELC |= BIT4;

    ADC14->CTL0 |= ADC14_CTL0_SSEL__MCLK | ADC14_CTL0_CONSEQ_2 | ADC14_CTL0_MSC | ADC14_CTL0_SHP | ADC14_CTL0_SHT0__32;
    ADC14->CTL1 |= ADC14_CTL1_RES__14BIT;
    ADC14->MCTL[0] |= ADC14_MCTLN_INCH_1 | ADC14_MCTLN_VRSEL_1;
    ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_ON | ADC14_CTL0_SC;

    /* TIMER_A0
     * Configured as PWM output on Pin 2.4.
     * Period = 20s
     * Minimum pulse width = 15.625ms
     */
    P2->SEL0 |= BIT4;
    P2->DIR |= BIT4;

    TIMER_A0->CCTL[1] |= TIMER_A_CCTLN_OUTMOD_7 | TIMER_A_CCTLN_CM__NONE;
    TIMER_A0->CCTL[1] &= ~TIMER_A_CCTLN_CAP;
    TIMER_A0->CCR[0] = (uint16_t) 1280;
    TIMER_A0->CCR[1] = (uint16_t) 128;
    TIMER_A0->CTL |= TIMER_A_CTL_SSEL__ACLK | TIMER_A_CTL_ID__8 | TIMER_A_CTL_CLR;

    /* PORT 4
     * P4.0 - P4.2 used for buttons to select set-point temperature and start sous vide machine.
     * A button press pulls the pin to Vdd. A 0->1 transition triggers a Port 4 interrupt.
     */
    P4->DIR &= ~(BIT0 | BIT1 | BIT2);
    P4->OUT &= ~(BIT0 | BIT1 | BIT2);
    P4->REN |= BIT0 | BIT1 | BIT2;
    P4->IES &= ~(BIT0 | BIT1 | BIT2);
    P4->IE = BIT0 | BIT1 | BIT2;
    P4->IFG = 0;
    NVIC->ISER[1] = 1 << ((PORT4_IRQn) & 31);

    /* SYSTICK
     * Interrupts every 5s to recompute the PID heater output.
     */
    SysTick->LOAD = 0x00E4E1C0 - 1; // 3MHz clk = 5 second countdown
    SysTick->VAL = BIT0;
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk;
    //SysTick_CTRL_ENABLE_Msk

    // Initialize PID variables.
    lastTemp = 0;
    temp = 0;
    setPointTemp = 55;

    computeInterval = 30; // 30s
    kp = 1;
    ki = 1/computeInterval;
    kd = 1/computeInterval;
    integralTerm = 0;
    output = 0;

    start = false;

    showChar('5' char5);
    showChar('5' char6);
//    SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;    // Enable sleep on exit from ISR
//    __DSB();

    __enable_irq();

    while (1)
    {
        float reading = sampleToTemp();
        char ones, tens, decimal;
        ones = ((int) reading) % 10;
        decimal = ((int) (reading*10)) % 10;
        tens = ((int) (reading/10)) % 10;

        showChar(tens, char1);
        showChar(ones, char2);
        showChar(decimal, char3);

        volatile int i;
        for (i = 1500000; i > 0; i--);
    }
}

/* Converts the ADC's sample of the TMP36G temperature sensor
 * TMP36G temperature curve: t = 100V - 50 *C.
 * ADC14's config. gives this eq: Nadc = 16384 * V / 1.45
 * Subbing and solving: t = Nadc * 145 / 16384 - 50 *C.
 */
float sampleToTemp(void)
{
    return ADC14->MEM[0] / 16384.0 * 145.0 - 50;
}

void SysTick_Handler(void)
{
    P1->OUT ^= BIT0;                        // Toggle P1.0 LED
}

void PORT4_IRQHandler(void)
{
    uint8_t flag = P4->IV;
    if (flag & BIT0){
        start ^= true;

        // Disable changing set point temperature when machine running.
        if (start){
            P4->IE = BIT0;
            P4->IFG &= ~(BIT1 | BIT2);
        }else{
            P4->IE = BIT0 | BIT1 | BIT2;
        }
    }else if (flag & BIT1){
        setPointTemperature += 1;
    }else if (flag & BIT2){
        setPointTemperature -= 1;
    }
}
