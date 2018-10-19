/*
 * lcd.h
 *
 *  Created on: Oct 17, 2018
 *      Author: Nicholas
 */

#ifndef LCD_H_
#define LCD_H_

#include<stdint.h>
#include"msp.h"

#define char1 16    // Digit A1 - L16
#define char2 32    // Digit A2 - L32
#define char3 40    // Digit A3 - L40
#define char4 36    // Digit A4 - L36
#define char5 28    // Digit A5 - L28
#define char6 44    // Digit A6 - L44

// LCD memory map for numeric digits (Byte Access)
extern const char digit[10][4];

void initLCD();

void showChar(char, int);


#endif /* LCD_H_ */
