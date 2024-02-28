/* Title: Semester Project 2 Group 3 - Main Code
 * Institution: University of Southern Denmark (SDU)
 * File: main.c
 * Author: Bence Toth
 * Year: 2024
 * Date: 28/02/2024
 * Semester: 2nd
 * Course: BEng in Electronics
 */

// Set the Clock Frequency to 16 MHz
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

// Constants
#define ROW_LEN 21
#define NUM_LEN 10
#define ADC_PIN PC1
#define IRQ_PIN PD2

// Include necessary libraries
#include <stdio.h>
#include <avr/io.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "i2cmaster.h"
#include "lm75.h"
#include "twi.h"
#include "ssd1306.h"

// I2C Communication Pins:   SCL -> PC5 | SDA -> PC4

// Function prototypes
int16_t main ( void ) ;
void initADC ( void ) ;
uint16_t readADC ( const uint8_t channel ) ;
uint32_t map ( uint32_t x , uint32_t in_min , uint32_t in_max , uint32_t out_min , uint32_t out_max ) ;
void addChars ( char *first , char *second , char *return_str ) ;
void printLine ( uint32_t value , char *text , uint8_t x_pos , uint8_t y_pos ) ;
ISR ( INT0_vect ) ;   // Interrupt Service Routine for PD2

// Variables
volatile uint8_t count = 0 , toggle = 1 ;
char TEXT_BUFFER [ ROW_LEN ] ;
char NUM_BUFFER [ NUM_LEN ] ;
char MESSAGE [ ROW_LEN ] = "                    " ;

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

void addChars ( char *first , char *second , char *return_str ) {
  uint8_t idx = 0 ;
  char blanks [ ] = "     " ;
  
  // You have to provide a semicolon character in the message, as it is used as the divider
  while ( first [ idx ] != ':' ) { idx++ ; }

  // Copy the contents of the two char arrays into the buffer
  memcpy ( return_str , first , strlen ( first ) ) ;
  memcpy ( &return_str [ idx + 1 ] , &blanks [ 0 ] , 1 ) ;      // Add a blank character between displayed text and value
	memcpy ( &return_str [ idx + 2 ] , second , strlen ( second ) ) ;   // Add the numerical value to the char array
  memcpy ( &return_str [ idx + strlen ( second ) + 2 ] , blanks , strlen ( blanks ) ) ;   // Add some blank space characters at the end
}

void printLine ( uint32_t value , char *text , uint8_t x_pos , uint8_t y_pos ) {
  // Use when displaying characters as well as numerical values
  itoa ( value , NUM_BUFFER , 10 ) ;
  memcpy ( MESSAGE , text , ROW_LEN ) ;
  addChars ( MESSAGE , NUM_BUFFER , TEXT_BUFFER ) ;
  SSD1306_SetPosition ( x_pos , y_pos ) ;
  SSD1306_DrawString ( TEXT_BUFFER , NORMAL ) ;
}

ISR ( INT0_vect ) {
  count++ ;
  toggle = 1 ;
  if ( count == 0xFF ) count = 0x00 ;
}

int16_t main ( void ) {

  DDRD &= ~( 1 << IRQ_PIN ) ;   // Set up IRQ_PIN as Input
  PORTD |= ( 1 << IRQ_PIN ) ;   // Enable Pull-up Resistor at IRQ_PIN
  EICRA |= ( 1 << ISC01 ) ;     // Trigger interrupt on falling-edge
  EIMSK |= ( 1 << INT0 ) ;      // Enable external interrupt INT0

  // Initializing the display and ADC
  initADC ( ) ;
  SSD1306_Init ( ) ;
  SSD1306_ClearScreen ( ) ;

  sei ( ) ; // Enable global interrupts

  // Infinite loop
  while ( 1 ) {
    if ( toggle ) {
      printLine ( count , "Counter:" , 0 , 0 ) ;
      toggle = 0 ;
    }
  }
  
  return 0 ;
}