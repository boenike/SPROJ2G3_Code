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
#define ADC_PIN PC0

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
#include "ssd1306.h"      // SSD1306 v3.0.1 library by Matiasus
#include "oled_wrapper.h"

// I2C Communication Pins:   SCL -> PC5 | SDA -> PC4

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