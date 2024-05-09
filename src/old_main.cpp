/* Title: Semester Project 2 Group 3 - Main Code
 * Institution: University of Southern Denmark (SDU)
 * Campus: Sonderborg
 * File: main.cpp
 * Author: Bence Toth
 * Date: 07/05/2024
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
#include <math.h>
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
#define GRAPH_MAX_VERT_POS 0.0F
#define GRAPH_MIN_VERT_POS 63.0F
#define NUM_DATAPOINTS 20
//#define F_CPU 16000000UL -> Only define outside of PlatformIO

String MSG ;
uint16_t potval = 0 ;
volatile uint8_t menu_checker = 0 , select = 0 , switched = 0 , clear_en = 0 , graph_en = 1 , snapshot_en = 0 , clear_graph_en = 0 ;
double temp_analog = 0.0 , temp_digital = 0.0 ;
const double freezer = 30.0 ;
double datapoints [ NUM_DATAPOINTS ] ;
uint8_t current_datapoint_pos = 0 , saved_datapoints = 0 ;

// I2C Communication Pins:   SCL -> PC5 | SDA -> PC4

void clearDataPoints ( double *data , uint8_t data_length ) {
  uint8_t ctr ;
  for ( ctr = 0 ; ctr < data_length ; ctr++ ) {
    data [ ctr ] = 0.0 ;
  }
}

double convertInterval ( double x , double in_min , double in_max , double out_min , double out_max ) {
  return ( x - in_min ) * ( out_max - out_min ) / ( in_max - in_min ) + out_min ; }

void drawBoundary ( void ) {
  ssd1306_drawHLine ( 0 , 0 , OLED_WIDTH - 1 ) ; // Top
  ssd1306_drawHLine ( 0 , OLED_HEIGHT - 1 , OLED_WIDTH - 1 ) ; // Bottom
  ssd1306_drawVLine ( 0 , 0 , OLED_HEIGHT - 1 ) ; // Left
  ssd1306_drawVLine ( OLED_WIDTH - 1 , 0 , OLED_HEIGHT - 1 ) ; // Right
  ssd1306_printFixed ( 30 , 0 , " SPROJ2EEG3 " , STYLE_BOLD ) ;
}

void findSmallestAndBiggest ( double *data , double *min , double *max , uint16_t stop_pos ) {
  uint8_t ctr ;
  double smallest = data [ 0 ] , biggest = data [ 0 ] ;
  for ( ctr = 0 ; ctr < stop_pos ; ctr++ ) {
    if ( data [ ctr ] < smallest ) smallest = data [ ctr ] ;
    if ( data [ ctr ] > biggest ) biggest = data [ ctr ] ;
  }
  * min = smallest ;
  * max = biggest ;
}

void drawDataPoints ( double *data , uint8_t stop_position ) {
  uint8_t ctr , x1 = 3 , x2 , y1 , y2 , offset ;
  double MIN , MAX ;
  findSmallestAndBiggest ( datapoints , &MIN , &MAX , saved_datapoints ) ;

  offset = (uint8_t) floor ( (double) OLED_WIDTH / stop_position ) ;
  x2 = offset ;
  y1 = (uint8_t) convertInterval ( data [ 0 ] , MIN , MAX , GRAPH_MIN_VERT_POS , GRAPH_MAX_VERT_POS ) ;

  for ( ctr = 0 ; ctr < stop_position ; ctr++ ) {
    y2 = (uint8_t) convertInterval ( data [ ctr ] , MIN , MAX , GRAPH_MIN_VERT_POS , GRAPH_MAX_VERT_POS ) ;
    ssd1306_drawLine ( x1 , y1 , x2 , y2 ) ;
    x1 = x2 ;
    y1 = y2 ;
    x2 += offset ;
  }
}

void shiftDataPoints ( double *data , uint8_t data_length , double new_data ) {
  uint8_t ctr ;
  for ( ctr = 0 ; ctr < ( data_length - 1 ) ; ctr++ ) {
    data [ ctr ] = data [ ctr + 1 ] ;
  }
  data [ data_length - 1 ] = new_data ;
}

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

void realTimeData ( void ) {
  temp_digital = get_temperature ( ) ;
  potval = read_ADC ( Temp_Input ) ;
  temp_analog = convertInterval ( (double) potval , 0.0 , 1023.0 , 35.0 , 42.0 ) ;

  MSG = "Armpit temp.: " + String ( temp_analog ) + "   " ;
  ssd1306_printFixed ( 3 , 24 , MSG.c_str() , STYLE_NORMAL ) ;

  MSG = "Finger temp.: " + String ( temp_digital ) + "  " ;
  ssd1306_printFixed ( 3 , 40 , MSG.c_str() , STYLE_NORMAL ) ;
}

ISR ( INT0_vect ) {
  if ( !select ) {
    menu_checker = !menu_checker ;
    switched = 0 ;
  }

  if ( select && !menu_checker ) snapshot_en = 1 ;

  if ( select && menu_checker ) {
    clear_graph_en = 1 ;
    graph_en = 1 ;
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

  clearDataPoints ( datapoints , NUM_DATAPOINTS ) ;

  ssd1306_128x64_i2c_init ( ) ;
  ssd1306_clearScreen ( ) ;
  ssd1306_positiveMode ( ) ;
  ssd1306_setFixedFont ( ssd1306xled_font6x8 ) ;
  drawBoundary ( ) ;
}

void loop ( void ) {
  EFontStyle real_style , graph_style ;

  if ( clear_en ) {
    clear_en = 0 ;
    ssd1306_clearScreen ( ) ;
    drawBoundary ( ) ;
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

    ssd1306_printFixed ( 18 , 16 , "Real-time data" , real_style ) ;
    ssd1306_printFixed ( 18 , 40 , "Heartrate graph" , graph_style ) ;
  }

  if ( select ) {

    if ( menu_checker && graph_en ) {
      graph_en = 0 ;
      if ( clear_graph_en ) {
        clear_graph_en = 0 ;
        clearDataPoints ( datapoints , NUM_DATAPOINTS ) ;
        saved_datapoints = 0 ;
        current_datapoint_pos = 0 ;
      }

      ssd1306_clearScreen ( ) ;
      drawDataPoints ( datapoints , saved_datapoints ) ;
    }

    if ( !menu_checker ) {
      realTimeData ( ) ;

      if ( snapshot_en ) { 
        snapshot_en = 0 ;

        if ( saved_datapoints < NUM_DATAPOINTS ) {
          datapoints [ current_datapoint_pos ] = temp_analog ;
          current_datapoint_pos++ ;
          saved_datapoints++ ;
        }

        else if ( saved_datapoints >= NUM_DATAPOINTS ) {
          shiftDataPoints ( datapoints , NUM_DATAPOINTS , temp_analog ) ;
        }
      }
    }
  }
}