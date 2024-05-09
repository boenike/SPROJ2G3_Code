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
 * I2C Communication Pins:   SCL -> PC5 | SDA -> PC4
 */

// Include necessary libraries
#include <stdio.h>
#include <avr/io.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <ssd1306.h>
#include "i2cmaster.h"
#include "lm75.h"

#ifndef F_CPU
#warning "CPU clock was not defined!"
#define F_CPU 16000000UL
// 16 MHz -> Only define outside of PlatformIO
#endif

#define Onboard_LED PB5
#define Heart_Enable PB1

#define Temp_Input PC1
#define Bat_Input PC2

#define Left_Button PD2
#define Right_Button PD3
#define Heart_Input PD4     // Using Timer0 for counting the input pulses

#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define GRAPH_MAX_VERT_POS 0.0F
#define GRAPH_MIN_VERT_POS 63.0F
#define NUM_DATAPOINTS 20
#define MAX_CHARS_PER_NUM 5
#define ADC_MAX_VAL 1023.0F
#define ADC_MIN_VAL 0.0F
#define ARMPIT_MIN_TEMP 35.0F
#define ARMPIT_MAX_TEMP 42.0F
#define DECIMAL_PRECISION 2
#define NUM_BASE 10

volatile uint8_t menu_checker = 0 , select = 0 , switched = 0 ;
volatile uint8_t clear_en = 0 , graph_en = 1 , snapshot_en = 0 , clear_graph_en = 0 ;
double datapoints [ NUM_DATAPOINTS ] ;
uint8_t current_datapoint_pos = 0 , saved_datapoints = 0 , no_data_flag = 0 ;

void setup_Counter0 ( void ) {
  TCCR0A = 0x00 ; // Normal mode counter with a TOP value of 0xFF
   // External Clock Source on pin T0 (PD4) triggered at a Falling edge
  TCCR0B = ( 1 << CS02 ) | ( 1 << CS01 ) ;
}

void convertDoubleToStr( double num , char *text ) {
  dtostrf(num , MAX_CHARS_PER_NUM , DECIMAL_PRECISION , text ) ;
}

void convertIntToStr ( int num , char *text ) {
  itoa ( num , text , NUM_BASE ) ;
}

uint8_t clearDataPoints ( double *data , uint8_t data_length ) {
  uint8_t ctr ;
  for ( ctr = 0 ; ctr < data_length ; ctr++ ) {
    data [ ctr ] = 0.0 ;
  }
  return 1 ;
}

double convertInterval ( double x , double in_min , double in_max , double out_min , double out_max ) {
  return ( x - in_min ) * ( out_max - out_min ) / ( in_max - in_min ) + out_min ; }

void drawBoundary ( void ) {
  ssd1306_drawHLine ( 0 , 0 , OLED_WIDTH - 1 ) ; // Top
  ssd1306_drawHLine ( 0 , OLED_HEIGHT - 1 , OLED_WIDTH - 1 ) ; // Bottom
  ssd1306_drawVLine ( 0 , 0 , OLED_HEIGHT - 1 ) ; // Left
  ssd1306_drawVLine ( OLED_WIDTH - 1 , 0 , OLED_HEIGHT - 1 ) ; // Right
  ssd1306_printFixed ( 26 , 0 , " Phobiameter " , STYLE_BOLD ) ;
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
  DDRB = ( 1 << Heart_Enable ) | ( 1 << Onboard_LED ) ;   // Enable pin as Output for the PMOS
  PORTB = ( 1 << Heart_Enable ) ;  // Turn off at first

  DDRC = 0xF0 ;  // Declare Analogue pins as Inputs
  PORTC = 0x30 ; // Set SDA & SCL pins to HIGH for I2C communication

  // Declare the stated pins as Inputs
  DDRD = ~( ( 1 << Left_Button ) | ( 1 << Right_Button ) | ( 1 << Heart_Input ) ) ;
  PORTD = ( 1 << Left_Button ) | ( 1 << Right_Button ) ;
  // Enable the internal pull-up resistors at the Buttons
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

void realTimeData ( double *finger_temp , double *armpit_temp ) {
  uint16_t potval ;
  char NUM_BUFFER [ MAX_CHARS_PER_NUM ] ;
  double temp_analog , temp_digital ;
  
  temp_digital = get_temperature ( ) ;
  potval = read_ADC ( Temp_Input ) ;
  temp_analog = convertInterval ( (double) potval , ADC_MIN_VAL , ADC_MAX_VAL , ARMPIT_MIN_TEMP , ARMPIT_MAX_TEMP ) ;

  convertDoubleToStr ( temp_analog , NUM_BUFFER ) ;
  ssd1306_printFixed ( 90 , 24 , NUM_BUFFER , STYLE_NORMAL ) ;

  convertDoubleToStr ( temp_digital , NUM_BUFFER ) ;
  ssd1306_printFixed ( 90 , 40 , NUM_BUFFER , STYLE_NORMAL ) ;

  ssd1306_printFixed ( 3 , 24 , "Armpit t.: " , STYLE_NORMAL ) ;
  ssd1306_printFixed ( 3 , 40 , "Finger t.: " , STYLE_NORMAL ) ;

  * finger_temp = temp_digital ;
  * armpit_temp = temp_analog ;
}

ISR ( INT0_vect ) {
  switch ( select ) {
    case 0 :
      menu_checker = !menu_checker ;
      switched = 0 ;
      break ;
    case 1 :
      switch ( menu_checker ) {
        case 0 :
          snapshot_en = 1 ;
          break ;
        case 1:
          clear_graph_en = 1 ;
          graph_en = 1 ;
          break ;
        default :
          break ;
      }
      break ;
    default :
      break ;
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

int main ( void ) {

  EFontStyle real_style , graph_style ;
  double temp_armpit , temp_fingertip ;

  init_Pins ( ) ; 
  init_External_Interrupts ( ) ;
  init_ADC ( ) ;
  i2c_init ( ) ;
  lm75_init ( ) ;

  no_data_flag = clearDataPoints ( datapoints , NUM_DATAPOINTS ) ;

  ssd1306_128x64_i2c_init ( ) ;
  ssd1306_clearScreen ( ) ;
  ssd1306_positiveMode ( ) ;
  ssd1306_setFixedFont ( ssd1306xled_font6x8 ) ;
  drawBoundary ( ) ;

  while ( 1 ) {

    if ( clear_en ) {
      clear_en = 0 ;
      ssd1306_clearScreen ( ) ;
      drawBoundary ( ) ;
    }

    switch ( select ) {

      case 0 :

        if ( !switched ) {
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
          ssd1306_printFixed ( 18 , 24 , "Real-time data" , real_style ) ;
          ssd1306_printFixed ( 18 , 40 , "Heartrate graph" , graph_style ) ;
        }
        break ;

      case 1 :

        switch ( menu_checker ) {

          case 0 :

            realTimeData ( &temp_fingertip , &temp_armpit ) ;

            if ( snapshot_en ) { 
              snapshot_en = 0 ;
              no_data_flag = 0 ;
              if ( saved_datapoints < NUM_DATAPOINTS ) {
                datapoints [ current_datapoint_pos ] = temp_armpit ;
                current_datapoint_pos++ ;
                saved_datapoints++ ;
              }

              else shiftDataPoints ( datapoints , NUM_DATAPOINTS , temp_armpit ) ;
            }
            break ;

          case 1 :

            if ( graph_en ) {
              graph_en = 0 ;

              if ( clear_graph_en ) {
                clear_graph_en = 0 ;
                no_data_flag = clearDataPoints ( datapoints , NUM_DATAPOINTS ) ;
                saved_datapoints = 0 ;
                current_datapoint_pos = 0 ;
              }

              ssd1306_clearScreen ( ) ;

              switch ( no_data_flag ) {
                case 0 :
                  drawDataPoints ( datapoints , saved_datapoints ) ;
                  break ;
                case 1 :
                  ssd1306_printFixed ( 21 , 24 , "GRAPH CLEARED" , STYLE_BOLD ) ;
                  ssd1306_printFixed ( 7 , 40 , "NO DATA TO DISPLAY" , STYLE_BOLD ) ;
                  break ;
                default :
                  break ;
              }
            }
            break ;

          default :
            break ;
        }

        break ;

      default :
        break ;
    }
  }
}