/*
 * max7219_Yncrea2.c
 *
 *  Created on: Apr 22, 2020
 *      Author: ThibautDELERUYELLE
 */
/*
*********************************************************************************************************
* Module     : MAX7219.C
* Author     : Randy Rasa Modified by Thibaut DELERUYELLE
* Description: MAX7219 LED Display Driver Routines
*
*  The Maxim MAX7219 is an LED display driver thant can control up to 64 individual LEDs, or
*  eight 7-segment LED digits, or any combination of individual LEDs and digits.  It frees the
*  host from the chore of constantly multiplexing the 8 rows and 8 columns.  In addition, it
*  takes care of brightness control (16 steps), and implements display test and display blank
*  (shutdown) features.
*
*  The host communicates with the MAX7219 using three signals: DATA, CLK, and LOAD.  This
*  modules bit-bangs them, but Motorola's SPI interface (or similar interface from other
*  manufacturers) may also be used to simplify and speed up the data transfer.
*                   ___ ___ ___ ___ ___ ___ ___ ___ ___ ___ ___ ___ ___ ___ ___ ___
*    DATA _________|D15|D14|D13|D12|D11|D10|D09|D08|D07|D06|D05|D04|D03|D02|D01|D00|______
*         ________    __    __    __    __    __    __    __    __    __    __    ________
*    CLK          |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|
*                __________________________________________________________________    ___
*    LOAD ______|                                                                  |__|
*
*
*  Implementation Notes:
*
*  1. This module was written and tested using an Atmel AT89C2051 microcontroller, with the
*     MAX7219 connected to I/O pins P3.3 (LOAD), P3.4 (CLK), and P3.3 (DATA).
*
*  2. Macros are provided to simplify control of the DATA, CLK, and LOAD signals.  You may also use
*     memory-mapped output ports such as a 74HC374, but you'll need to replace the macros with
*     functions, and use a shadow register to store the state of all the output bits.
*
*  3. This module was tested with the evaluation version of Hi-Tech C-51, using the small memory model.
*     It should be portable to most other compilers, with minor modifications.
*
*  4. The MAX7219 is configured for "no decode" mode, rather than "code B" decoding.  This
*     allows the program to display more than the 0-9,H,E,L,P that code B provides.  However,
*     the "no decode" method requires that each character to be displayed have a corresponding
*     entry in a lookup table, to convert the ascii character to the proper 7-segment code.
*     MAX7219_LookupCode() provides this function, using the MAX7219_Font[] array.  If you need
*     to display more than the characters I have provided, simply add them to the table ...
*
*********************************************************************************************************
*/


/*
*********************************************************************************************************
* Include Header Files
*********************************************************************************************************
*/
//#include  <8051.h>                                    // microcontroller header file
#include "main.h"
#include "max7219_Yncrea2.h"                                  // MAX7219 header file
extern SPI_HandleTypeDef hspi1;
//need to be defined in main.h
/*#define SPI_SCK_Pin
#define SPI_SCK_GPIO_Port
#define SPI_MISO_Pin
#define SPI_MISO_GPIO_Port
#define SPI_MOSI_Pin
#define SPI_MOSI_GPIO_Port
#define SPI_CS_Pin
#define SPI_CS_GPIO_Port
*/

/*
*********************************************************************************************************
* Constants
*********************************************************************************************************
*/
#define REG_DECODE        0x09                        // "decode mode" register
#define REG_INTENSITY     0x0a                        // "intensity" register
#define REG_SCAN_LIMIT    0x0b                        // "scan limit" register
#define REG_SHUTDOWN      0x0c                        // "shutdown" register
#define REG_DISPLAY_TEST  0x0f                        // "display test" register

#define INTENSITY_MIN     0x00                        // minimum display intensity
#define INTENSITY_MAX     0x0f                        // maximum display intensity


/*
*********************************************************************************************************
* Macros
*********************************************************************************************************
*/
//#define DATA_PORT     P3                              // assume "DATA" is on P3.5
//#define DATA_DDR      P3
//#define DATA_BIT      0x20
//#define DATA_0()      (DATA_PORT &= ~DATA_BIT)
//#define DATA_1()      (DATA_PORT |=  DATA_BIT)
//#define CLK_PORT      P3                              // assume "CLK" is on P3.4
//#define CLK_DDR       P3
//#define CLK_BIT       0x10
//#define CLK_0()       (CLK_PORT &= ~CLK_BIT)
//#define CLK_1()       (CLK_PORT |=  CLK_BIT)
//#define LOAD_PORT     P3                              // assume "LOAD" is on P3.3
//#define LOAD_DDR      P3
//#define LOAD_BIT      0x08
//#define LOAD_0()      (LOAD_PORT &= ~LOAD_BIT)
//#define LOAD_1()      (LOAD_PORT |=  LOAD_BIT)
#define LOAD_0()      SPI_CS_High()
#define LOAD_1()      SPI_CS_Low()


 void SPI_CS_High()
{

	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
}

 void SPI_CS_Low()
{

	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
}

/*
*********************************************************************************************************
* Private Data
*********************************************************************************************************
*/
// ... none ...


/*
*********************************************************************************************************
* Private Function Prototypes
*********************************************************************************************************
*/
static void MAX7219_Write (unsigned char reg_number, unsigned char data);
static void MAX7219_SendByte (unsigned char data);
static unsigned char MAX7219_LookupCode (char character);


// ...................................... Public Functions ..............................................


/*
*********************************************************************************************************
* MAX7219_Init()
*
* Description: Initialize MAX7219 module; must be called before any other MAX7219 functions.
* Arguments  : none
* Returns    : none
*********************************************************************************************************
*/
void MAX7219_Init (void)
{
  //DATA_DDR |= DATA_BIT;                               // configure "DATA" as output
  //CLK_DDR  |= CLK_BIT;                                // configure "CLK"  as output
  //LOAD_DDR |= LOAD_BIT;                               // configure "LOAD" as output

  MAX7219_Write(REG_SCAN_LIMIT, 7);                   // set up to scan all eight digits
  MAX7219_Write(REG_DECODE, 0x00);                    // set to "no decode" for all digits
  MAX7219_ShutdownStop();                             // select normal operation (i.e. not shutdown)
  MAX7219_DisplayTestStop();                          // select normal operation (i.e. not test mode)
  MAX7219_Clear();                                    // clear all digits
  MAX7219_SetBrightness(INTENSITY_MAX);               // set to maximum intensity
}


/*
*********************************************************************************************************
* MAX7219_ShutdownStart()
*
* Description: Shut down the display.
* Arguments  : none
* Returns    : none
*********************************************************************************************************
*/
void MAX7219_ShutdownStart (void)
{
  MAX7219_Write(REG_SHUTDOWN, 0);                     // put MAX7219 into "shutdown" mode
}


/*
*********************************************************************************************************
* MAX7219_ShutdownStop()
*
* Description: Take the display out of shutdown mode.
* Arguments  : none
* Returns    : none
*********************************************************************************************************
*/
void MAX7219_ShutdownStop (void)
{
  MAX7219_Write(REG_SHUTDOWN, 1);                     // put MAX7219 into "normal" mode
}


/*
*********************************************************************************************************
* MAX7219_DisplayTestStart()
*
* Description: Start a display test.
* Arguments  : none
* Returns    : none
*********************************************************************************************************
*/
void MAX7219_DisplayTestStart (void)
{
  MAX7219_Write(REG_DISPLAY_TEST, 1);                 // put MAX7219 into "display test" mode
}


/*
*********************************************************************************************************
* MAX7219_DisplayTestStop()
*
* Description: Stop a display test.
* Arguments  : none
* Returns    : none
*********************************************************************************************************
*/
void MAX7219_DisplayTestStop (void)
{
  MAX7219_Write(REG_DISPLAY_TEST, 0);                 // put MAX7219 into "normal" mode
}


/*
*********************************************************************************************************
* MAX7219_SetBrightness()
*
* Description: Set the LED display brightness
* Arguments  : brightness (0-15)
* Returns    : none
*********************************************************************************************************
*/
void MAX7219_SetBrightness (char brightness)
{
  brightness &= 0x0f;                                 // mask off extra bits
  MAX7219_Write(REG_INTENSITY, brightness);           // set brightness
}


/*
*********************************************************************************************************
* MAX7219_Clear()
*
* Description: Clear the display (all digits blank)
* Arguments  : none
* Returns    : none
*********************************************************************************************************
*/
void MAX7219_Clear (void)
{
  char i;
  for (i=0; i < 8; i++)
    MAX7219_Write(i, 0x00);                           // turn all segments off
}


/*
*********************************************************************************************************
* MAX7219_DisplayChar()
*
* Description: Display a character on the specified digit.
* Arguments  : digit = digit number (0-7)
*              character = character to display (0-9, A-Z)
* Returns    : none
*********************************************************************************************************
*/
void MAX7219_DisplayChar (char digit, char character)
{
  MAX7219_Write(digit, MAX7219_LookupCode(character));
}


// ..................................... Private Functions ..............................................


/*
*********************************************************************************************************
* LED Segments:         a
*                     ----
*                   f|    |b
*                    |  g |
*                     ----
*                   e|    |c
*                    |    |
*                     ----  o dp
*                       d
*   Register bits:
*      bit:  7  6  5  4 | 3  2  1  0
*           dp  a  b  c | d  e  f  g
*********************************************************************************************************
*/
static const struct {
	char   ascii;
	char   segs;
} MAX7219_Font[] = {
  {' ', 0x80},
  {'0', 0x7e},
  {'1', 0x30},
  {'2', 0x6d},
  {'3', 0x79},
  {'4', 0x33},
  {'5', 0x5b},
  {'6', 0x5f},
  {'7', 0x70},
  {'8', 0x7f},
  {'9', 0x7b},
  {'A', 0x77},
  {'B', 0x1f},
  {'C', 0x4e},
  {'D', 0x3d},
  {'E', 0x4f},
  {'F', 0x47},
  {'H', 0x37}, //add ok
  {'I', 0x30},
  {'J', 0x38}, //add ok
  {'L', 0x0e}, //add ok
  {'N', 0x76}, //modified
  {'O', 0x1d}, //add ok
  {'P', 0x67}, //add ok
  {'Q', 0x73}, //add ok
  {'R', 0x05}, //add ok
  {'S', 0x5b},
  {'T', 0x0f}, //add ok
  {'U', 0x3e}, //modified
//  {'\0', 0x80} //modified (dp)
};



/*
*********************************************************************************************************
* MAX7219_LookupCode()
*
* Description: Convert an alphanumeric character to the corresponding 7-segment code.
* Arguments  : character to display
* Returns    : segment code
*********************************************************************************************************
*/
static unsigned char MAX7219_LookupCode (char character)
{
  char i;
  for (i = 0; MAX7219_Font[i].ascii; i++)             // scan font table for ascii code
    if (character == MAX7219_Font[i].ascii)
      return MAX7219_Font[i].segs;                    // return segments code
  return 0;                                           // code not found, return null (blank)
}


/*
*********************************************************************************************************
* MAX7219_Write()
*
* Description: Write to MAX7219
* Arguments  : reg_number = register to write to
*              dataout = data to write to MAX7219
* Returns    : none
*********************************************************************************************************
*/
static void MAX7219_Write (unsigned char reg_number, unsigned char dataout)
{
  LOAD_1();                                           // take LOAD high to begin
  MAX7219_SendByte(reg_number);                       // write register number to MAX7219
  MAX7219_SendByte(dataout);                          // write data to MAX7219
  LOAD_0();                                           // take LOAD low to latch in data
  LOAD_1();                                           // take LOAD high to end
}


/*
*********************************************************************************************************
* MAX7219_SendByte()
*
* Description: Send one byte to the MAX7219
* Arguments  : dataout = data to send
* Returns    : none
*********************************************************************************************************
*/
/*
static void MAX7219_SendByte (unsigned char dataout)
{
  char i;
  for (i=8; i>0; i--) {
    unsigned char mask = 1 << (i - 1);                // calculate bitmask
    CLK_0();                                          // bring CLK low
    if (dataout & mask)                               // output one data bit
      DATA_1();                                       //  "1"
    else                                              //  or
      DATA_0();                                       //  "0"
    CLK_1();                                          // bring CLK high
	}
}*/
static void MAX7219_SendByte (unsigned char dataout)
{
	HAL_SPI_Transmit(&hspi1, &dataout, 1, 1000);
}


