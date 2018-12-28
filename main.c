#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "msp.h"
#include "pid.h"

enum characters {char1 = 16, char2 = 32, char3 = 40, char4 = 36, char5 = 28, char6 = 44};

#define INITIALSETPOINT 55

const int outLower = 128, outUpper = 1152, pwmPeriod = 1280;
const int Kp = 600, Ki = 60, Kd = 60, pidComputeInterval = 60;


float setPointTemperature;
int output;
bool start;

const char digit[10][4] =
{
    {0xC, 0xF, 0x8, 0x2},  /* "0" LCD segments a+b+c+d+e+f+k+q */
    {0x0, 0x6, 0x0, 0x2},  /* "1" */
    {0xB, 0xD, 0x0, 0x0},  /* "2" */
    {0x3, 0xF, 0x0, 0x0},  /* "3" */
    {0x7, 0x6, 0x0, 0x0},  /* "4" */
    {0x7, 0xB, 0x0, 0x0},  /* "5" */
    {0xF, 0xB, 0x0, 0x0},  /* "6" */
    {0x4, 0xE, 0x0, 0x0},  /* "7" */
    {0xF, 0xF, 0x0, 0x0},  /* "8" */
    {0x7, 0xF, 0x0, 0x0}   /* "9" */
};

float sampleToTemp (void);
void showDig(int c, enum characters position);

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
	 *      Clocks ADC14, TIMER_32, and ARM processor.
	 */
    CS->KEY = CS_KEY_VAL;
    CS->CTL1 |= CS_CTL1_DIVA__64;

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
    TIMER_A0->CCR[0] = (uint16_t) pwmPeriod;
    TIMER_A0->CCR[1] = (uint16_t) outLower;
    TIMER_A0->CTL |= TIMER_A_CTL_SSEL__ACLK | TIMER_A_CTL_ID__8 | TIMER_A_CTL_CLR;

    /* PORT 4
     * P4.1 - P4.3 used for buttons to select set-point temperature and start sous vide machine.
     * A button press pulls the pin to Vdd. A 0->1 transition triggers a Port 4 interrupt.
     */
    P4->DIR &= ~(BIT1 | BIT2 | BIT3);
    P4->OUT &= ~(BIT1 | BIT2 | BIT3);
    P4->REN |= BIT1 | BIT2 | BIT3;
    P4->IES &= ~(BIT1 | BIT2 | BIT3);
    P4->IE = BIT1 | BIT2 | BIT3;
    P4->IFG = 0;
    NVIC->ISER[1] = 1 << ((PORT4_IRQn) & 31);

    /* LCD_F
     * 4-mux mode
     * LCD refresh frequency = 64Hz
     * Blinking frequency = 1Hz
     */
    P3->SEL1 |= 0xF2;
    P6->SEL1 |= 0x0C;
    P7->SEL1 |= 0xF0;
    P8->SEL1 |= 0xFC;
    P9->SEL1 |= 0xFF;
    P10->SEL1 |= 0x3F;
    LCD_F->PCTL0 |= 0xFC0F004F;
    LCD_F->PCTL1 |= 0x0000FFFF;

    LCD_F->CTL |= LCD_F_CTL_SSEL_3 | LCD_F_CTL_DIV_15 | LCD_F_CTL_PRE_2 | LCD_F_CTL_MX_3 | LCD_F_CTL_SON;
    LCD_F->BMCTL |= LCD_F_BMCTL_CLRBM | LCD_F_BMCTL_CLRM | LCD_F_BMCTL_BLKDIV_7 | LCD_F_BMCTL_BLKPRE_3;
    LCD_F->BMCTL |= LCD_F_BMCTL_BLKMOD_3;
    LCD_F->CSSEL0 |= 0x0C000048;
    LCD_F->M[26] = 0x01;
    LCD_F->M[27] = 0x02;
    LCD_F->M[6] = 0x04;
    LCD_F->M[3] = 0x08;
    LCD_F->CTL |= LCD_F_CTL_ON;


    /* TIMER_32
     * Interrupts every pidComputeInterval seconds to recompute the PID heater output.
     */
    TIMER32_1->LOAD = (uint32_t)pidComputeInterval * 3000000 - 1;
    TIMER32_1->BGLOAD = (uint32_t)pidComputeInterval * 3000000 - 1;
    TIMER32_1->CONTROL |= TIMER32_CONTROL_IE | TIMER32_CONTROL_MODE | TIMER32_CONTROL_SIZE;
    NVIC->ISER[0] = 1 << ((T32_INT1_IRQn) & 31);
    //SysTick->VAL = BIT0;
    //SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk;

    // Initialize PID variables.
    start = false;
    setPointTemperature = INITIALSETPOINT;
    output = outLower;

    setOutputLimits (outLower, outUpper);
    setComputeInterval (pidComputeInterval);
    setGains (Kp, Ki, Kd);

    showDig((int)(setPointTemperature / 10) % 10, char5);
    showDig((int)setPointTemperature % 10, char6);
//    SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;    // Enable sleep on exit from ISR
//    __DSB();

    __enable_irq();

    while (1)
    {
        float temperature = sampleToTemp() + 0.05; // Rnd to nearest tenth.
        int ones = (int)temperature % 10;
        int tens = (int)(temperature / 10) % 10;
        int decimal = (int)(temperature * 10) % 10;

        showDig (tens, char1);
        showDig (ones, char2);
        showDig (decimal, char3);
        volatile int i;
        for (i = 3000000; i > 0; i--);
    }
}

/* Converts the ADC's sample of the TMP36G temperature sensor
 * TMP36G temperature curve: t = 100V - 50 *C.
 * ADC14's config. gives this eq: Nadc = 16384 * V / 1.45
 * Subbing and solving: t = Nadc * 145 / 16384 - 50 *C.
 */
float sampleToTemp(void)
{
    int sample = ADC14->MEM[0];
    float temperature = (float) sample * 0.00885009765625 - 50;
    return temperature;
}

void showDig(int c, enum characters position)
{
    int i;
    if (c < 0 || c > 9)
    {
        for (i=0; i<4; i++)
        {
            LCD_F->M[position+i] = 0x00;
        }
    } else {
        for (i=0; i<4; i++)
        {
            LCD_F->M[position+i] = digit[c][i];
        }
    }
}

void T32_INT1_IRQHandler(void)
{
    TIMER32_1->INTCLR = 0;
    output = compute(sampleToTemp());
    TIMER_A0->CCR[1] = (uint16_t) output;
}

void PORT4_IRQHandler(void)
{
    uint8_t flag = P4->IV;
    if (flag == 0x4){
        start ^= true;

        if (start){
            // Disable changing set point temperature when machine running.
            P4->IE = BIT1;
            P4->IFG &= ~(BIT2 | BIT3);

            // Disable LCD blinking to signify heater start.
            LCD_F->BMCTL &= ~0x2;

            initializePID (sampleToTemp());
            setSetpoint (setPointTemperature);
            TIMER_A0->CCR[1] = (uint16_t) outLower;

            // Start systick and timer_a0.
            TIMER_A0->CTL |= TIMER_A_CTL_MC__UP;
            TIMER32_1->LOAD = (uint32_t)pidComputeInterval * 3000000 - 1;
            TIMER32_1->CONTROL |= TIMER32_CONTROL_ENABLE;
            //SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
        }else{
            // Disable systick and timer_a0
            TIMER32_1->CONTROL &= ~TIMER32_CONTROL_ENABLE;
            //SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
            TIMER_A0->CTL &= ~TIMER_A_CTL_MC__UP;

            // Enable interrupts on P4.1 & P4.2, and blink LCD.
            P4->IE = BIT1 | BIT2 | BIT3;
            LCD_F->BMCTL |= LCD_F_BMCTL_BLKMOD_3;
        }
    }else if (flag == 0x6){
        setPointTemperature += 1;
        setPointTemperature = (int)setPointTemperature % 100;
        showDig((int)(setPointTemperature / 10) % 10, char5);
        showDig((int)setPointTemperature % 10, char6);
    }else if (flag == 0x8){
        setPointTemperature -= 1;
        if (setPointTemperature < 0)
            setPointTemperature = 99;
        showDig((int)(setPointTemperature / 10) % 10, char5);
        showDig((int)setPointTemperature % 10, char6);
    }
}
