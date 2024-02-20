/* Title: Semester Project 2 Group 3 - Main Code
 * Institution: University of Southern Denmark (SDU)
 * File: main.c
 * Author: Bence Toth
 * Year: 2024
 * Semester: 2nd
 * Course: BEng in Electronics
 */

// Constants
#define row_len 21
#define num_len 10
#define adChannel PC1
#define base 10

#include <stdio.h>
#include <avr/io.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <util/delay.h>
#include "twi.h"
#include "ssd1306.h"

// SCL -> PC5 | SDA -> PC4

// Function prototypes
int16_t main ( void ) ;
void initADC ( void ) ;
uint16_t readADC ( const uint8_t channel ) ;
uint32_t map ( uint32_t x , uint32_t in_min , uint32_t in_max , uint32_t out_min , uint32_t out_max ) ;
void addChars ( char *first , uint8_t size_first , char *second , uint8_t size_second , char *return_str ) ;
void printLine ( uint32_t value , char *text , uint8_t x_pos , uint8_t y_pos ) ;

// Variables
char TEXT_BUFFER [ row_len ] ;
char NUM_BUFFER [ num_len ] ;
char MESSAGE [ row_len ] = "                    " ;
uint16_t adcVal ;
uint32_t percentage ;

void initADC ( void ) {
  ADMUX = 0x00 ;                            // Clear at start
  ADMUX |= ( 1 << REFS0 ) ;                 // Set reference voltage to AVcc
  ADCSRA |= ( 1 << ADEN ) | ( 1 << ADPS2 ) | ( 1 << ADPS1 ) | ( 1 << ADPS0 ) ;
  // Enable ADC and set prescaler to 128
}

uint16_t readADC ( const uint8_t channel ) {
	ADMUX |= ( channel & 0x07 ) ;               // cut off channel value to the limited bits available
	ADCSRA |= ( 1 << ADSC ) ;                   // Start ADC conversion
  while ( ADCSRA & ( 1 << ADSC ) ) ;          // Wait while the conversion completes
  return ( ( uint16_t ) ADCL ) + ( ( uint16_t ) ADCH << 8 ) ;  // Bit-shift High nibble to get the result of the conversion
}

uint32_t map ( uint32_t x , uint32_t in_min , uint32_t in_max , uint32_t out_min , uint32_t out_max ) {
    return ( x - in_min ) * ( out_max - out_min ) / ( in_max - in_min ) + out_min ;
}

void addChars ( char *first , uint8_t size_first , char *second , uint8_t size_second , char *return_str ) {
  uint8_t idx = 0 ;
  char blanks [ ] = "     " ;
  
  while ( first [ idx ] != ':' ) { // You have to provide a semicolon character in the message, as it is used as the divider
    idx++ ;
  }

  memcpy ( return_str , first , size_first ) ;
	memcpy ( &return_str [ idx + 1 ] , second , size_second ) ;
  memcpy ( &return_str [ idx + strlen ( second ) + 1 ] , blanks , sizeof ( blanks ) ) ;
}

void printLine ( uint32_t value , char *text , uint8_t x_pos , uint8_t y_pos ) {
  // Use when displaying characters as well as numerical values
  itoa ( value , NUM_BUFFER , base ) ;
  memcpy ( MESSAGE , text , row_len ) ;
  addChars ( MESSAGE , row_len , NUM_BUFFER , num_len , TEXT_BUFFER ) ;
  SSD1306_SetPosition ( x_pos , y_pos ) ;
  SSD1306_DrawString ( TEXT_BUFFER , NORMAL ) ;
}

int16_t main ( void ) {

  // Initializing the display and ADC
  initADC ( ) ;
  SSD1306_Init ( ) ;
  SSD1306_ClearScreen ( ) ;

  // Infinite loop
  while ( 1 ) {
    adcVal = readADC ( adChannel ) ;
    percentage = map ( adcVal , 0 , 1023 , 0 , 100 ) ;
    printLine ( adcVal , "ADC Value:" , 0 , 0 ) ;
    printLine ( percentage , "Percentage:" , 0 , 3 ) ;
  }
  return 0 ;
}