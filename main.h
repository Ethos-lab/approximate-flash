/*
 * main.h
 *
 *  Created on: Jan 11, 2016
 *      Author: amir
 */

#include <stdint.h>

#ifndef MAIN_H_
#define MAIN_H_

int main(void);
void SysTick_Handler(void);
void LCD_TEST_LOOP(void);
void Delay(uint32_t dlyTicks);

void SWO_SetupForPrint(void);
int _write(int file, const char *ptr, int len);


#endif /* MAIN_H_ */
