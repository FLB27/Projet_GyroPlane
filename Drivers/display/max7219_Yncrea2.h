/*
 * max7219_Yncrea2.h
 *
 *  Created on: Apr 22, 2020
 *      Author: ThibautDELERUYELLE
 */

#ifndef MAX7219_YNCREA2_H_
#define MAX7219_YNCREA2_H_

void MAX7219_Init(void);
void MAX7219_ShutdownStart(void);
void MAX7219_ShutdownStop(void);
void MAX7219_DisplayTestStart(void);
void MAX7219_DisplayTestStop(void);
void MAX7219_SetBrightness(char brightness);
void MAX7219_Clear(void);
void MAX7219_DisplayChar(char digit, char character);

//define by TDE
 void SPI_CS_High();
 void SPI_CS_Low();

#endif /* MAX7219_YNCREA2_H_ */
