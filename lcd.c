/*
 * lcd.c
 *
 *  Created on: Oct 17, 2018
 *      Author: Nicholas
 */

#include "lcd.h"

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

void initLCD()
{
    // Select LCD function on port pins being used for LCD
    P3->SEL1 |= 0xF2;
    P6->SEL1 |= 0x0C;
    P7->SEL1 |= 0xF0;
    P8->SEL1 |= 0xFC;
    P9->SEL1 |= 0xFF;
    P10->SEL1 |= 0x3F;

    // Unlock CS module for register access
    CS->KEY = CS_KEY_VAL;
    CS->CTL1 = CS_CTL1_SELA__LFXTCLK | CS_CTL1_SELS__DCOCLK | CS_CTL1_SELM__DCOCLK;

    // Configure LCD_F Port Control Register to enable each LCD pin that is in use
    LCD_F->PCTL0 |= 0xFC0F004F;   // L0-L3, L6, L16-L19, L26-L31
    LCD_F->PCTL1 |= 0x0000FFFF;   // L32-L47

    // Configure LCD_F Control Register. ACLK = LCD clock source, Divider = 16, Pre-scaler = 4, 4-mux mode
    LCD_F->CTL |= LCD_F_CTL_SSEL_0 | LCD_F_CTL_DIV_15 | LCD_F_CTL_PRE_2 | LCD_F_CTL_MX_3 | LCD_F_CTL_SON;

    // Configure Blinking and Memory Control Register. Clear all blinking and main memory, Blinking frequency = 1Hz.
    LCD_F->BMCTL |= LCD_F_BMCTL_CLRBM | LCD_F_BMCTL_CLRM | LCD_F_BMCTL_BLKDIV_7 | LCD_F_BMCTL_BLKPRE_3;
    LCD_F->BMCTL |= LCD_F_BMCTL_BLKMOD_1;    // Enable Blinking Mode of segments selected in both LCDMx & LCDBMx.

    // Configure COM pins
    LCD_F->CSSEL0 |= 0x0C000048;  // Select Pins L3, L6, L26, L27 as common lines
    LCD_F->M[26] = 0x01;          // COM0
    LCD_F->M[27] = 0x02;          // COM1
    LCD_F->M[6] = 0x04;           // COM2
    LCD_F->M[3] = 0x08;           // COM3

    LCD_F->CTL |= LCD_F_CTL_ON;
}

void showChar(char c, int position)
{
    int i;
    if (c == ' ')
    {
        for (i=0; i<4; i++)
        {
            LCD_F->M[position+i] = 0x00;
        }
    }
    else if (c >= '0' && c <= '9')
    {
        for (i=0; i<4; i++)
        {
            LCD_F->M[position+i] = digit[c-48][i];
        }
    }
    else if (c >= 'A' && c <= 'Z')
    {
        for (i=0; i<4; i++)
        {
            LCD_F->M[position+i] = alphabetBig[c-65][i];
        }
    }
    else
    {
        LCD_F->M[position] = 0x00;
    }
}
