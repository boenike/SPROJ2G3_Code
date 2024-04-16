/* Title: Semester Project 2 Group 3 - Main Code
 * Institution: University of Southern Denmark (SDU)
 * File: main.c
 * Author: Bence Toth
 * Year: 2024
 * Date: 16/04/2024
 * Semester: 2nd
 * Course: BEng in Electronics
 * Display: 0.96" SSD1306 OLED (128x64) via I2C
 */

// Constants
#define ROW_LEN 21
#define NUM_LEN 6
#define ADC_PIN PC0
#define OLED_ADDR 0x7B

// Include necessary libraries
#include <stdio.h>
#include <avr/io.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include "ssd1306.h"      // SSD1306 v3.0.1 by Matiasus

// I2C Communication Pins:   SCL -> PC5 | SDA -> PC4

typedef enum { INTEGER , DOUBLE } numtype_t ;
typedef union { double dbl ; uint32_t intgr ; } number_t ;

void initADC ( void ) {
  ADMUX = ( 1 << REFS0 ) ;                 // Set reference voltage to AVcc
  ADCSRA |= ( 1 << ADEN ) | ( 1 << ADPS2 ) | ( 1 << ADPS1 ) | ( 1 << ADPS0 ) ;
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

void printLineWithNumber ( char *text , uint8_t x_pos , uint8_t y_pos , numtype_t NUMTYPE , number_t *NUMBER ) {
  // Use when displaying characters as well as numerical values

  char TEXT_BUFFER [ ROW_LEN ] , NUM_BUFFER [ NUM_LEN ] ;

  switch ( NUMTYPE ) {
   case INTEGER : itoa ( (int) NUMBER->intgr , NUM_BUFFER , 10 ) ; break ;
   case DOUBLE : dtostrf ( NUMBER->dbl , 5 , 2 , NUM_BUFFER ) ; break ;
   default : break ;
  }
  
  addChars ( text , NUM_BUFFER , TEXT_BUFFER ) ;
  SSD1306_SetPosition ( x_pos , y_pos ) ;
  SSD1306_DrawString ( TEXT_BUFFER , NORMAL ) ;
}

int main ( void ) {

  DDRC = ~( 1 << ADC_PIN ) ;    // Define the ADC Pin as Input

  number_t pctg , value ;                // ADC Reading

  // Initializing the display and ADC
  initADC ( ) ;
  SSD1306_Init ( ) ;
  SSD1306_ClearScreen ( ) ;
  SSD1306_NormalScreen ( ) ;
  SSD1306_SetPosition ( 0 , 0 ) ;
  SSD1306_DrawString ( " SPROJ2EEG3" , BOLD ) ;

  // Infinite loop
  while ( 1 ) {
    value.intgr = (uint32_t) readADC ( ADC_PIN ) ;
    pctg.dbl = ( (double) value.intgr * 100.0 ) / 1023.0 ;   // Percentage

    printLineWithNumber ("Reading: " , 0 , 50 , INTEGER , &value ) ;
    printLineWithNumber ("Percentage: " , 0 , 70 , DOUBLE , &pctg ) ;
    _delay_ms ( 100.0 ) ;
  }
}