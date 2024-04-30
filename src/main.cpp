/* Title: Semester Project 2 Group 3 - Main Code
 * Institution: University of Southern Denmark (SDU)
 * Campus: Sonderborg
 * File: main.cpp
 * Author: Bence Toth
 * Date: 30/04/2024
 * Course: BEng in Electronics
 * Semester: 2nd
 * Display: 0.96" SSD1306 OLED (128x64) interfaced via I2C
 * Library: https://github.com/lexus2k/ssd1306
 * Dependency Graph:
    |-- ssd1306 @ 1.8.5
    |-- SPI @ 1.0
    |-- Wire @ 1.0
 */

// Include necessary libraries
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <ssd1306.h>
#include <stdio.h>
#include <avr/io.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include "i2cmaster.h"
#include "lm75.h"

#define ADC_Pin PC0
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

String MSG ;
uint16_t potval ;
volatile uint8_t butval = 0 ;
double pctg , temp ;

// I2C Communication Pins:   SCL -> PC5 | SDA -> PC4
// ssd1306_drawLine(x1,y1,x2,y2)

/* To print out numbers with strings:
      - 1. Declare a String variable,
      - 2. Modify it with the desired chars and nums,
      - 3. Convert String object to c-style char array,
      - For example: String msg ;
      -              msg = "Hello " + String ( value ) ;
      -              const char *text = msg.c_str ( ) ;
 */

void init_External_Interrupts ( void ) {
  PCICR = ( 1 << PCIE1 ) ;
  PCMSK1 = ( 1 << PCINT9 ) ;
  //EICRA = ( 1 << ISC11 ) | ( 1 << ISC01 ) ;
  //EIMSK = ( 1 << INT1 ) | ( 1 << INT0 ) ;
  sei ( ) ;       // Enable global interrupts 
  // Trigger an interrupt at a falling edge
  // At both INT0 and INT1
}

ISR ( PCINT1_vect ) {
   if ( ( PINC & ( 1 << PC1 ) ) == 0 ) {
      butval = !butval ;
      if ( butval ) PORTB |= ( 1 << PB5 ) ;
      else PORTB = 0x00 ;
   }
}

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

double __map ( double x , double in_min , double in_max , double out_min , double out_max ) {
  return ( x - in_min ) * ( out_max - out_min ) / ( in_max - in_min ) + out_min ; }

void drawBoundary ( void ) {
  ssd1306_drawLine ( 0 , 0 , OLED_WIDTH - 1 , 0 ) ; // Top
  ssd1306_drawLine ( OLED_WIDTH - 1 , 0 , OLED_WIDTH - 1 , OLED_HEIGHT - 1 ) ; // Right
  ssd1306_drawLine ( 0 , OLED_HEIGHT - 1 , OLED_WIDTH - 1 , OLED_HEIGHT - 1 ) ; // Bottom
  ssd1306_drawLine ( 0 , 0 , 0 , OLED_HEIGHT - 1 ) ; // Left
}

void setup ( void ) {

  DDRC = 0xF0 ;   // Declare the Buttons as Inputs
  PORTC = ( 1 << PC1 ) ;  // Enable internal pull-up resistors at the Button

  DDRD = 0xFF ;   // Declare the LEDs as Outputs
  PORTD = 0x00 ;  // At first, turn off all the LEDs
  
  DDRB = 0xFF ;   // On-board LED as Output

  initADC ( ) ;
  init_External_Interrupts ( ) ;
  i2c_init ( ) ;
  lm75_init ( ) ;

  ssd1306_setFixedFont ( ssd1306xled_font6x8 ) ;
  ssd1306_128x64_i2c_init ( ) ;
  ssd1306_clearScreen ( ) ;
  ssd1306_positiveMode ( ) ;
  drawBoundary ( ) ;
  ssd1306_printFixed ( 30 , 5 , " SPROJ2EEG3 " , STYLE_BOLD ) ;
}

void loop ( void ) {
  temp = get_temperature ( ) ;
  potval = readADC ( ADC_Pin ) ;
  pctg = __map ( (double) potval , 0.0 , 1023.0 , 0.0 , 100.0 ) ;

  MSG = "ADC value: " + String ( potval ) + "   " ;
  ssd1306_printFixed ( 3 , 18 , MSG.c_str() , STYLE_NORMAL ) ;

  MSG = "Percentage: " + String ( pctg ) + "   " ;
  ssd1306_printFixed ( 3 , 25 , MSG.c_str() , STYLE_NORMAL ) ;

  MSG = "Temperature: " + String ( temp ) + "  " ;
  ssd1306_printFixed ( 3 , 32 , MSG.c_str() , STYLE_NORMAL ) ;

  _delay_ms ( 50.0 ) ;
}