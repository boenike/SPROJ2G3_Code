/* Title: Semester Project 2 Group 3 - Main Code
 * Institution: University of Southern Denmark (SDU)
 * File: main.c
 * Author: Bence Toth
 * Year: 2024
 * Date: 19/04/2024
 * Semester: 2nd
 * Course: BEng in Electronics
 * Display: 0.96" SSD1306 OLED (128x64) via I2C
 */

// Constants
#define ROW_LEN 21
#define NUM_LEN 6
#define DEC_PRECISION 2
#define RADIX 10
#define ADC_PIN PC0
#define OLED_ADDR 0x7B

// Include necessary libraries
#include <stdio.h>
#include <avr/io.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
//#include <avr/interrupt.h>
//#include <avr/eeprom.h>
#include <util/delay.h>
#include "i2cmaster.h"
#include "lm75.h"
#include "ssd1306.h"      // SSD1306 v3.0.1 by Matiasus

// I2C Communication Pins:   SCL -> PC5 | SDA -> PC4

typedef enum { INTEGER , DOUBLE } numtype_t ;
typedef enum { BEFORE , AFTER } numplacement_t ;
typedef union { double dbl ; int32_t intgr ; } number_t ;

void initADC ( void ) {
  ADMUX = ( 1 << REFS0 ) ;    // Set reference voltage to AVcc
  ADCSRA = ( 1 << ADEN ) | ( 1 << ADPS2 ) | ( 1 << ADPS1 ) | ( 1 << ADPS0 ) ;
  // Enable ADC and set prescaler to 128
}

uint16_t readADC ( const uint8_t channel ) {
	ADMUX |= ( channel & 0x07 ) ;               // Cut off channel value to the limited bits available
	ADCSRA |= ( 1 << ADSC ) ;                   // Start ADC conversion
  while ( ADCSRA & ( 1 << ADSC ) ) ;          // Wait while the conversion completes
  return ( ( uint16_t ) ADCL + ( ( uint16_t ) ADCH << 8 ) ) ;  // Bit-shift High nibble to get the result of the conversion
}

uint32_t map ( uint32_t x , uint32_t in_min , uint32_t in_max , uint32_t out_min , uint32_t out_max ) {
    return ( x - in_min ) * ( out_max - out_min ) / ( in_max - in_min ) + out_min ;
}

void printNumber ( uint8_t x_pos , uint8_t y_pos , numtype_t NUMTYPE , number_t *NUMBER , enum E_Font font ) {
  char NUM_BUFFER [ NUM_LEN ] ;

  switch ( NUMTYPE ) {
   case INTEGER : itoa ( NUMBER->intgr , NUM_BUFFER , RADIX ) ; break ;
   case DOUBLE : dtostrf ( NUMBER->dbl , NUM_LEN , DEC_PRECISION , NUM_BUFFER ) ; break ;
   default : break ;
  }

  SSD1306_SetPosition ( x_pos , y_pos ) ;
  SSD1306_DrawString ( NUM_BUFFER , font ) ;
}

void printString ( char *text , uint8_t x_pos , uint8_t y_pos , enum E_Font font ) {
  SSD1306_SetPosition ( x_pos , y_pos ) ;
  SSD1306_DrawString ( text , font ) ;
}

void addChars ( char *first , char *second , char *return_str ) {
  uint8_t ctr , idx = strlen ( first ) , offset = strlen ( second ) , remainder = strlen ( return_str ) - idx - offset ;
  
  // Copy the contents of the two char arrays into the buffer
  memcpy ( return_str , first , idx ) ;
	memcpy ( &return_str [ idx ] , second , offset ) ;       // Add the numerical value to the char array

  if ( remainder > 0 ) {
    char blanks [ remainder ] ;
    for ( ctr = 0 ; ctr < remainder ; ctr++ ) {
      blanks [ ctr ] = ' ' ;
    }
    memcpy ( &return_str [ idx + offset ] , blanks , remainder ) ;   // Add some blank space characters at the end
  }
}

void printStringWithNumber ( char *text , uint8_t x_pos , uint8_t y_pos , numtype_t NUMTYPE , number_t *NUMBER , enum E_Font font , numplacement_t placement ) {
  // Use when displaying characters as well as numerical values

  char TEXT_BUFFER [ strlen ( text ) + NUM_LEN ] , NUM_BUFFER [ NUM_LEN ] ;

  switch ( NUMTYPE ) {
   case INTEGER : itoa ( NUMBER->intgr , NUM_BUFFER , RADIX ) ; break ;
   case DOUBLE : dtostrf ( NUMBER->dbl , NUM_LEN , DEC_PRECISION , NUM_BUFFER ) ; break ;
   default : break ;
  }

  switch ( placement ) {
    case BEFORE : addChars ( NUM_BUFFER , text , TEXT_BUFFER ) ; break ;
    case AFTER : addChars ( text , NUM_BUFFER , TEXT_BUFFER ) ; break ;
    default : break ;
  }
  
  SSD1306_SetPosition ( x_pos , y_pos ) ;
  SSD1306_DrawString ( TEXT_BUFFER , font ) ;
}

int main ( void ) {

  DDRC = ~( 1 << ADC_PIN ) ;    // Define the ADC Pin as Input

  number_t value , pctg , temp ;
  const double freeze = 100.0 ;

  // Initializing the display and ADC
  initADC ( ) ;
  i2c_init ( ) ;
  lm75_init ( ) ;
  SSD1306_Init ( ) ;
  SSD1306_ClearScreen ( ) ;
  SSD1306_NormalScreen ( ) ;

  printString ( " SPROJ2EEG3" , 0 , 0 , BOLD ) ;
  printString ( "ADC Value: " , 0 , 10 , NORMAL ) ;
  printString ( "Percentage: " , 0 , 20 , NORMAL ) ;
  printString ( "Temperature: " , 0 , 30 , NORMAL ) ;
  
  // Infinite loop
  while ( 1 ) {
    value.intgr = (int32_t) readADC ( ADC_PIN ) ;
    pctg.dbl = ( (double) value.intgr * 100 ) / 1023 ;
    temp.dbl = get_temperature ( ) ;

    printStringWithNumber ( "     " , 80 , 10 , INTEGER , &value , NORMAL , BEFORE ) ;
    printStringWithNumber ( "  " , 80 , 20 , DOUBLE , &pctg , NORMAL , BEFORE ) ;
    printNumber ( 80 , 30 , DOUBLE , &temp , NORMAL ) ;

    _delay_ms ( freeze ) ;
  }
}