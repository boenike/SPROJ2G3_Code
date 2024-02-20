/* Title: Semester Project 2 Group 3 - Main Code
 * Institution: University of Southern Denmark (SDU)
 * File: main.c
 * Author: Bence Toth
 * Year: 2024
 * Semester: 2nd
 * Course: BEng in Electronics
 */

// Definitions for the preprocessor
#define row_len 21
#define num_len 10
#define adChannel PC0
#define base 10

// Including the necessary libraries
#include <stdio.h>
#include <avr/io.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <util/delay.h>
#include "twi.h"
#include "ssd1306.h"

// For I2C Comm.: SCL -> PC5 | SDA -> PC4

// Variables
char message [ ] = " ADC Value: " ;
char TEXT_BUFFER [ row_len ] ;
char NUM_BUFFER [ num_len ] ;
uint8_t clear_offset ;
uint16_t adcVal ;

// Function declarations
int16_t main ( void ) ;
void initADC ( void ) ;
uint16_t readADC ( const uint8_t channel ) ;
void addChars ( char *first , uint8_t size_first , char *second , uint8_t size_second , char *return_str ) ;

// Function descriptions
void initADC ( void ) {
  ADMUX = 0x00 ;                            // Clear ADC Multiplexer register at start
  ADMUX |= ( 1 << REFS0 ) ;                 // Set reference voltage to AVcc (5V)
  ADCSRA |= ( 1 << ADEN ) | ( 1 << ADPS2 ) | ( 1 << ADPS1 ) | ( 1 << ADPS0 ) ;
  // Enable ADC and set prescaler to 128
}

uint16_t readADC ( const uint8_t channel ) {
	ADMUX |= ( channel & 0x07 ) ;                           // Cut off selected channel value to the limited bits available
	ADCSRA |= ( 1 << ADSC ) ;                               // Start A/D conversion
  while ( ADCSRA & ( 1 << ADSC ) ) ;                      // Wait while the conversion completes
  uint16_t result =  ADCL + ( ( uint16_t ) ADCH << 8 ) ;  // Bit-shift High nibble to get the result of the conversion
  return result ;
}

void addChars ( char *first , uint8_t size_first , char *second , uint8_t size_second , char *return_str ) {
  memcpy ( return_str , first , size_first ) ;
	memcpy ( &return_str [ size_first -1 ] , second , size_second ) ;
}

int16_t main ( void ) {

  // Initializing the SSD1306 0.96" 128x64 OLED display and the ADC
  initADC ( ) ;
  SSD1306_Init ( ) ;
  SSD1306_ClearScreen ( ) ;

  while ( 1 ) {
    adcVal = readADC ( adChannel ) ;        // Read the analog input voltage
    itoa ( adcVal , NUM_BUFFER , base ) ;   // Convert numeric value to char array representation
    addChars ( message , sizeof ( message ) , NUM_BUFFER , sizeof ( NUM_BUFFER ) , TEXT_BUFFER ) ;

    SSD1306_SetPosition ( 0 , 0 ) ;
    SSD1306_DrawString ( TEXT_BUFFER , NORMAL ) ;

  }

  return 0 ;
}