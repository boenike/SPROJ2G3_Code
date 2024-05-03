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

#define Onboard_LED PB5
#define Heart_Enable PB1

#define Temp_Input PC1
#define Bat_Input PC2

#define Left_Button PD2
#define Right_Button PD3
#define Heart_Input PD4

#define OLED_WIDTH 128
#define OLED_HEIGHT 64
//#define F_CPU 16000000UL -> Only define outside of PlatformIO

String MSG ;
uint16_t potval ;
volatile uint8_t menu_checker = 0 , select = 0 , switched = 0 , clear_en = 0 , graph_en = 1 ;
double pctg , temp ;
const double freezer = 60.0 ;

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

/*ISR ( PCINT1_vect ) {
   if ( ( PINC & ( 1 << PC0 ) ) == 0 ) {
      butval = !butval ;
      if ( butval ) PORTB |= ( 1 << PB5 ) ;
      else PORTB = 0x00 ;
   }
}*/

void init_Pins ( void ) {
  DDRB = ( 1 << Onboard_LED ) | ( 1 << Heart_Enable ) ;   // On-board LED and EN pin as Outputs
  PORTB = 0x00 ;  // Turn off at first

  DDRC = 0xF0 ;  // Declare Analogue pins as Inputs
  PORTC = 0x30 ; // Set SDA & SCL pins to HIGH for I2C communication

  DDRD = ~(( 1 << Left_Button ) | ( 1 << Right_Button ) | ( 1 << Heart_Input )) ;   // Declare the stated pins as Inputs
  PORTD = ( 1 << Left_Button ) | ( 1 << Right_Button ) ;     // Enable internal pull-up resistors at the Buttons 
}

void init_External_Interrupts ( void ) {
  EICRA = ( 1 << ISC11 ) | ( 1 << ISC01 ) ;
  EIMSK = ( 1 << INT1 ) | ( 1 << INT0 ) ;
  sei ( ) ;       // Enable global interrupts 
  // Trigger an interrupt at a falling edge at both INT0 and INT1
}

void init_ADC ( void ) {
  ADMUX = ( 1 << REFS0 ) ;    // Set reference voltage to AVcc
  ADCSRA = ( 1 << ADEN ) | ( 1 << ADPS2 ) | ( 1 << ADPS1 ) | ( 1 << ADPS0 ) ;
  // Enable ADC and set prescaler to 128
}

uint16_t read_ADC ( const uint8_t channel ) {
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
  ssd1306_printFixed ( 30 , 5 , " SPROJ2EEG3 " , STYLE_BOLD ) ;
}

ISR ( INT0_vect ) {
  if ( !select ) {
    menu_checker = !menu_checker ;
    switched = 0 ;
  }
}

ISR ( INT1_vect ) {
  select = !select ;
  clear_en = 1 ;
  switch ( select ) {
    case 0 :
      switched = 0 ;
      break ;
    case 1 :
      graph_en = 1 ;
      break ;
    default :
      break ;
  }
}

void setup ( void ) {

  init_Pins ( ) ; 
  init_External_Interrupts ( ) ;
  init_ADC ( ) ;
  i2c_init ( ) ;
  lm75_init ( ) ;

  ssd1306_setFixedFont ( ssd1306xled_font6x8 ) ;
  ssd1306_128x64_i2c_init ( ) ;
  ssd1306_clearScreen ( ) ;
  ssd1306_positiveMode ( ) ;
  drawBoundary ( ) ;
}

void loop ( void ) {
  EFontStyle real_style , graph_style ;

  if ( clear_en ) {
    clear_en = 0 ;
    switch ( select ) {
      case 0 :
        ssd1306_printFixed ( 3 , 18 , "                    " , STYLE_NORMAL ) ;
        ssd1306_printFixed ( 3 , 25 , "                    " , STYLE_NORMAL ) ;
        ssd1306_printFixed ( 3 , 32 , "                    " , STYLE_NORMAL ) ;
        break ;
      case 1 :
        ssd1306_printFixed ( 3 , 25 , "                    " , STYLE_NORMAL ) ;
        ssd1306_printFixed ( 3 , 45 , "                    " , STYLE_NORMAL ) ;
        break ;
      default :
        break ;
    }
  }

  if ( !select && !switched ) {
    switched = 1 ;

    switch ( menu_checker ) {
      case 0 :
        real_style = STYLE_BOLD ;
        graph_style = STYLE_NORMAL ;
        break ;
      case 1 :
        real_style = STYLE_NORMAL ;
        graph_style = STYLE_BOLD ;
        break ;
      default :
        break ;
    }

    ssd1306_printFixed ( 18 , 25 , "Real-time data" , real_style ) ;
    ssd1306_printFixed ( 18 , 45 , "Heartrate graph" , graph_style ) ;
  }

  if ( select ) {

    if ( menu_checker && graph_en ) {
      graph_en = 0 ;
      ssd1306_printFixed ( 5 , 32 , "Under construction!" , STYLE_BOLD ) ;
    }

    if ( !menu_checker ) {
      temp = get_temperature ( ) ;
      potval = read_ADC ( Temp_Input ) ;
      pctg = __map ( (double) potval , 0.0 , 1023.0 , 0.0 , 100.0 ) ;

      MSG = "ADC value:   " + String ( potval ) + "   " ;
      ssd1306_printFixed ( 3 , 18 , MSG.c_str() , STYLE_NORMAL ) ;

      MSG = "Percentage:  " + String ( pctg ) + "   " ;
      ssd1306_printFixed ( 3 , 25 , MSG.c_str() , STYLE_NORMAL ) ;

      MSG = "Temperature: " + String ( temp ) + "  " ;
      ssd1306_printFixed ( 3 , 32 , MSG.c_str() , STYLE_NORMAL ) ;

      _delay_ms ( freezer ) ;
    }
  }
}