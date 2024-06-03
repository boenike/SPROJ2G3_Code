/* Title: Semester Project 2 Group 3 - Main Code
 * Institution: University of Southern Denmark (SDU)
 * Campus: Sonderborg
 * File: main.c
 * Author: Bence Toth
 * Date: 03/06/2024
 * Course: BEng in Electronics
 * Semester: 2nd
 * Display: 0.96" SSD1306 OLED (128x64) via I2C
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
#include <ssd1306.h>
#include "defines.h"
#include "i2cmaster.h"
#include "lm75.h"

// Global variables
volatile uint8_t timeout_checker = 0 , interval_check = 0 ;
volatile uint16_t flags = 0 , pulse_duration = 0 ;
double BPM = 0.0 , displayed_BPM = 0.0 , temp_diff = 0.0 ;
double pulse_datapoints [ NUM_DATAPOINTS ] , temp_datapoints [ NUM_DATAPOINTS ] ;
char text [ MAX_CHARS_IN_ROW ] ;
const char indicator [ ] = "->" ;
uint8_t current_datapoint_pos = 0 , saved_datapoints = 0 ;

ISR ( TIMER2_COMPA_vect )  {  // ISR for software debouncing
  timeout_checker++ ;
  if ( timeout_checker == DELAY_MULTIPLIER ) {
    flags |= BTN_BOUNCE_FLAG ; // Enable button flag
    timeout_checker = 0 ; // Reset timeout checking
    TCCR2B = 0x00 ;       // Stop Timer2
  }
}

ISR ( TIMER1_COMPA_vect ) {
  interval_check = 0 ; 
  TCCR1B &= ~( ( 1 << CS12 ) | ( 1 << CS10 ) ) ;   // Disable Timer1
  TCNT1 = 0x0000 ;
}

ISR ( TIMER0_COMPA_vect ) {
  interval_check++ ;
  switch ( interval_check ) {
    case 1 :
      // Start Timer1 with prescaler set to 1024
      TCNT1 = 0x0000 ;        // Reset Timer1 value
      TCCR1B |= ( 1 << CS12 ) | ( 1 << CS10 ) ;
      break ;
    case 2 :
      pulse_duration = TCNT1 ;
      TCCR1B &= ~( ( 1 << CS12 ) | ( 1 << CS10 ) ) ;   // Disable Timer1
      interval_check = 0 ;
      flags |= BPM_EN ; // Enable calculating new BPM value
      break ;
    default : break ;
  }
}

void setup_Timers ( void ) {
  // Timer0 is set up for counting external pulses at a Falling edge
  // Timer1 is set up for the heartrate measuring interval
  // Timer2 is set up for button debouncing purposes
  // Timer2 -> 15 millisecond interval for the button debounce
  // Calculated using formula: OCRn = (F_CPU/prescaler)*time_seconds - 1

  TCCR0A = ( 1 << WGM01 ) ;  // Counter0 in CTC Mode
  OCR0A = 1 ;                // Detect one pulse
  TCCR0B = 0x00 ;            // First, disable Counter0
  TIMSK0 = ( 1 << OCIE0A ) ;

  TCCR1A = 0x00 ;            // Timer1 in CTC Mode
  TCCR1B = ( 1 << WGM12 ) ;  // First, disable Timer1
  OCR1A = OCR1_VAL ;         // Set Timer1 COMPA to precalculated value

  TCCR2A = ( 1 << WGM21 ) ;  // Timer2 in CTC Mode
  TCCR2B = 0x00 ;            // First, disable Timer2
  OCR2A = OCR2_VAL ;         // Set Timer2 COMPA to precalculated value
  TIMSK2 = ( 1 << OCIE2A ) ; // Enable COMPA Interrupt

  pulse_duration = 0 ;       // At first, stop pulse count
}

void clearDataPoints ( double data [ ] , uint8_t data_length ) {
  for ( uint8_t ctr = 0 ; ctr < data_length ; ctr++ ) {
    data [ ctr ] = 0.0 ;
  }
}

uint8_t allDatapointsEqual ( double data [ ] , uint8_t stop_point ) {
  double reference = data [ 0 ] ;
  for ( uint8_t ctr = 1 ; ctr < stop_point ; ctr++ ) {
    if ( data [ ctr ] != reference ) return 0 ;
  }
  return 1 ;
}

double convertInterval ( double x , double in_min , double in_max , double out_min , double out_max ) {
  return ( x - in_min ) * ( out_max - out_min ) / ( in_max - in_min ) + out_min ;
}

void drawBoundary ( ) {
  ssd1306_drawHLine ( 0 , OLED_HEIGHT - 1 , OLED_WIDTH - 1 ) ; // Bottom
  ssd1306_drawHLine ( 0 , 0 , 45 ) ; // Top 1
  ssd1306_drawHLine ( 86 , 0 , OLED_WIDTH - 1 ) ; // Top 2
  ssd1306_drawVLine ( 0 , 0 , OLED_HEIGHT - 1 ) ; // Left
  ssd1306_drawVLine ( OLED_WIDTH - 1 , 0 , OLED_HEIGHT - 1 ) ; // Right

  switch ( flags & MENU_SELECTOR ) {
    case 0 :
      ssd1306_printFixed ( 45 , 0 , " Menu " , STYLE_NORMAL ) ; break ;
    case MENU_SELECTOR :
      ssd1306_printFixed ( 45 , 0 , " Data " , STYLE_NORMAL ) ; break ;
    default : break ;
  }
}

void printErrorMessage ( void ) {
  ssd1306_printFixed ( 5 , 30 , "LM75 inaccessible!" , STYLE_NORMAL ) ;
  ssd1306_printFixed ( 20 , 40 , "Please insert" , STYLE_NORMAL ) ;
  ssd1306_printFixed ( 5 , 50 , "external temp module" , STYLE_NORMAL ) ;
}

void findSmallestAndBiggest ( double data [ ] , double *min , double *max , uint8_t stop_pos ) {
  double smallest = data [ 0 ] , biggest = data [ 0 ] ;
  for ( uint8_t ctr = 0 ; ctr < stop_pos ; ctr++ ) {
    if ( data [ ctr ] < smallest ) smallest = data [ ctr ] ;
    if ( data [ ctr ] > biggest  ) biggest  = data [ ctr ] ;
  }

  * min = smallest ;
  * max = biggest ;
}

void drawDataPoints ( double data [ ] , uint8_t stop_position ) {
  if ( stop_position <= MIN_DATAPOINTS ) {
    ssd1306_printFixed ( 10 , 24 , "Min 2 datapoints" , STYLE_BOLD ) ;
    ssd1306_printFixed ( 10 , 32 , "have to be given!" , STYLE_BOLD ) ;
    return ;
  }

  uint8_t halver , ctr , x1 = 3 , x2 , y1 , y2 , offset ;
  double MIN , MAX ;

  switch ( allDatapointsEqual ( data , stop_position ) ) {
    case 0 : halver = 0  ; break ;
    case 1 : halver = 30 ; break ;
    default : break ;
  }
  
  findSmallestAndBiggest ( data , &MIN , &MAX , stop_position ) ;

  offset = (uint8_t) floor ( (double) OLED_WIDTH / stop_position ) ;
  x2 = offset ;
  y1 = (uint8_t) convertInterval ( data [ 0 ] , MIN , MAX , GRAPH_MIN_VERT_POS , GRAPH_MAX_VERT_POS ) - halver ;

  for ( ctr = 0 ; ctr < stop_position ; ctr++ ) {
    y2 = ( uint8_t ) convertInterval ( data [ ctr ] , MIN , MAX , GRAPH_MIN_VERT_POS , GRAPH_MAX_VERT_POS ) - halver ;
    ssd1306_drawLine ( x1 , y1 , x2 , y2 ) ;
    x1 = x2 ;
    y1 = y2 ;
    x2 += offset ;
  }

  switch ( flags & SWITCH_GRAPH ) {
    case 0 :
      snprintf ( text , MAX_CHARS_IN_ROW , "BPM:  %3.0f  |  %3.0f" , MIN , MAX ) ; break ;
    case SWITCH_GRAPH :
      snprintf ( text , MAX_CHARS_IN_ROW , "dT:  %4.2f  |  %4.2f" , MIN , MAX )  ; break ;
    default : break ;
  }
  ssd1306_printFixed ( 0 , 0 , text , STYLE_NORMAL ) ;
}

void shiftDataPoints ( double data [ ] , uint8_t data_length , double new_data ) {
  for ( uint8_t ctr = 0 ; ctr < ( data_length - 1 ) ; ctr++ ) {
    data [ ctr ] = data [ ctr + 1 ] ;
  }
  data [ data_length - 1 ] = new_data ;
}

void init_Pins ( void ) {
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
  sei ( ) ;   // Enable global interrupts
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

void realTimeData ( ) {
  uint16_t potval ;
  double temp_analog = 0.0 , temp_digital = 0.0 ;
  
  // Taking average samples
  for ( uint8_t ctr = 0 ; ctr < MAX_TEMP_SAMPLES ; ctr++ ) {
    temp_digital += get_temperature ( ) ;
    potval = read_ADC ( Temp_Input ) ;
    temp_analog += convertInterval ( ( ADC_MAX_VAL - ( double ) potval ) , ADC_MIN_VAL , ADC_MAX_VAL , ARMPIT_MIN_TEMP , ARMPIT_MAX_TEMP ) ;
  }

  // Averaging out the temperature values
  temp_digital /= MAX_TEMP_SAMPLES ;
  temp_analog /= MAX_TEMP_SAMPLES ;
  temp_diff = ( temp_analog - temp_digital ) >= 0.0 ? ( temp_analog - temp_digital ) : ( temp_digital - temp_analog ) ;

  ssd1306_printFixed ( 3 , 8 , "Last pulse:" , STYLE_NORMAL ) ;
  snprintf ( text , MAX_CHARS_IN_ROW , "Finger temp: %5.2f " , temp_digital ) ;
  ssd1306_printFixed ( 3 , 24 , text , STYLE_NORMAL ) ;
  snprintf ( text , MAX_CHARS_IN_ROW , "Armpit temp: %5.2f " , temp_analog ) ;
  ssd1306_printFixed ( 3 , 32 , text , STYLE_NORMAL ) ;
  snprintf ( text , MAX_CHARS_IN_ROW , "Temp. diff: % 5.2f " , temp_diff ) ;
  ssd1306_printFixed ( 3 , 48 , text , STYLE_NORMAL ) ;

  if ( flags & BPM_EN ) {   // Convert time period to BPM value if a reading is triggered
    flags &= ~BPM_EN ;
    BPM = ( 1000.0 * PULSE_BPM_MULTIPLIER ) / convertInterval ( ( double ) pulse_duration , 0.0 , ( double ) OCR1_VAL , 0.0 , PULSE_GATHER_INTERVAL ) ;
    if ( BPM >= BPM_LOWER_LIMIT && BPM <= BPM_UPPER_LIMIT ) {   // Check the timer value whether it is between the desired interval
      displayed_BPM = BPM ;
      flags |= FIRST_PULSE_MEASURED ;
      snprintf ( text , MAX_CHARS_IN_ROW , "Last pulse: %3.0f BPM " , displayed_BPM ) ;
      ssd1306_printFixed ( 3 , 8 , text , STYLE_NORMAL ) ;
    }
  }
}

ISR ( INT0_vect ) {
  if ( flags & BTN_BOUNCE_FLAG ) {

    flags &= ~BTN_BOUNCE_FLAG ; // Disable button flag
    // Set prescaler to 1024 and start Timer2
    TCCR2B = ( 1 << CS22 ) | ( 1 << CS21 ) | ( 1 << CS20 ) ;

    switch ( flags & SELECT ) {
      case 0 :
        flags = ( flags & MENU_CHECKER ) > 0 ? ( flags & ~MENU_CHECKER ) : ( flags | MENU_CHECKER ) ;
        flags &= ~SWITCHED ;
        break ;
      case SELECT :
        switch ( flags & MENU_CHECKER ) {
          case 0 :
            flags |= SNAPSHOT_EN ; break ;
          case MENU_CHECKER :
            flags |= GRAPH_EN ;
            flags = ( flags & SWITCH_GRAPH ) > 0 ? ( flags & ~SWITCH_GRAPH ) : ( flags | SWITCH_GRAPH ) ;
            break ;
          default : break ;
        }
        break ;
      default : break ;
    }
  }
}

ISR ( INT1_vect ) {
  if ( flags & BTN_BOUNCE_FLAG ) {
    flags &= ~BTN_BOUNCE_FLAG ; // Disable button flag
    // Set prescaler to 1024 and start Timer2
    TCCR2B = ( 1 << CS22 ) | ( 1 << CS21 ) | ( 1 << CS20 ) ;
    flags |= CLEAR_EN ;

    switch ( flags & SELECT ) {
      case 0 :
        flags |= SELECT  ;
        switch ( flags & MENU_CHECKER ) {
          case 0 :
            flags |= MENU_SELECTOR ;
            // Start pulse count
            // External Clock Source on PD4, triggered at a rising edge
            // Start Counter0 with prescalers set to 1024
            TCNT0 = 0x00 ;    // Reset Counter0 value
            TCCR0B = ( 1 << CS02 ) | ( 1 << CS01 ) | ( 1 << CS00 ) ;
            break ;
          case MENU_CHECKER : flags |= GRAPH_EN ; break ;
          default : break ;
        }
        break ;

      case SELECT :
        flags &= ~( SELECT | FIRST_PULSE_MEASURED | MENU_SELECTOR | SWITCHED ) ;
        // Stop pulse count
        TCCR0B = 0x00 ;                                  // Disable Counter0
        TCCR1B &= ~( ( 1 << CS12 ) | ( 1 << CS10 ) ) ;   // Disable Timer1
        pulse_duration = 0 ;
        break ;

      default : break ;
    }
  }
}

int main ( void ) {
  EFontStyle real_style , graph_style ;
  uint8_t menu_data_x , menu_graph_x , indicator_pos_y , LM75_ACCESSED ;
  //uint16_t bat_ADC_reading ;
  //double bat_pctg ;

  flags |= ( BTN_BOUNCE_FLAG | GRAPH_EN ) ; // Initialize wanted flags at the start

  init_Pins ( ) ; 
  setup_Timers ( ) ;
  init_External_Interrupts ( ) ;
  init_ADC ( ) ;

  ssd1306_128x64_i2c_init ( ) ;
  ssd1306_clearScreen ( ) ;
  ssd1306_positiveMode ( ) ;
  ssd1306_setFixedFont ( ssd1306xled_font6x8 ) ;
  drawBoundary ( ) ;

  LM75_ACCESSED = lm75_init ( ) ;   // Try to access the LM75 temperature sensor via I2C

  clearDataPoints ( pulse_datapoints , NUM_DATAPOINTS ) ;   // Reset the data holding arrays at the start
  clearDataPoints ( temp_datapoints , NUM_DATAPOINTS ) ;

  while ( 1 ) {

    if ( ( flags & CLEAR_EN ) && LM75_ACCESSED ) {
      flags &= ~CLEAR_EN ;
      ssd1306_clearScreen ( ) ;
      if ( !( flags & SWITCHED ) || ( flags & MENU_SELECTOR ) ) { drawBoundary ( ) ; }
    }

    switch ( flags & SELECT ) {
      case 0 :
        if ( !( flags & SWITCHED ) ) {
          flags |= SWITCHED ;

          /*bat_ADC_reading = read_ADC ( Bat_Input ) ;
          bat_pctg = ( 100 * convertInterval ( ( double ) bat_ADC_reading , ADC_MIN_VAL , BAT_MAX_READING , BAT_MIN_VOLTAGE , BAT_MAX_VOLTAGE ) ) / BAT_MAX_VOLTAGE ;
          snprintf ( text , MAX_CHARS_IN_ROW , "Bat. level: %.2f%%  " , bat_pctg ) ;
          ssd1306_printFixed ( 3 , 48 , text , STYLE_NORMAL ) ;*/

          if ( LM75_ACCESSED ) {
            switch ( flags & MENU_CHECKER ) {
              case 0 :
                real_style = STYLE_BOLD ;
                graph_style = STYLE_ITALIC ;
                indicator_pos_y = MENU_DATA_Y ;
                menu_data_x = START_POS_X + CHAR_WIDTH * sizeof ( indicator - 1 ) ;
                menu_graph_x = START_POS_X ;
                break ;
              case MENU_CHECKER :
                real_style = STYLE_ITALIC ;
                graph_style = STYLE_BOLD ;
                indicator_pos_y = MENU_GRAPH_Y ;
                menu_graph_x = START_POS_X + CHAR_WIDTH * sizeof ( indicator - 1 ) ;
                menu_data_x = START_POS_X ;
                break ;
              default : break ;
            }
          
            ssd1306_printFixed ( START_POS_X , indicator_pos_y , indicator , STYLE_NORMAL ) ;
            ssd1306_printFixed ( menu_data_x , MENU_DATA_Y , " Real-time data  " , real_style ) ;
            ssd1306_printFixed ( menu_graph_x , MENU_GRAPH_Y , " Graph plotting  " , graph_style ) ;
          }
          else { printErrorMessage ( ) ; }
        }
        break ;

      case SELECT :
        if ( LM75_ACCESSED ) {
          switch ( flags & MENU_CHECKER ) {
            case 0 :
              realTimeData ( ) ;

              if ( ( flags & SNAPSHOT_EN ) && ( flags & FIRST_PULSE_MEASURED ) ) {
                flags &= ~SNAPSHOT_EN ;

                if ( saved_datapoints < NUM_DATAPOINTS ) {
                  pulse_datapoints [ current_datapoint_pos ] = displayed_BPM ;
                  temp_datapoints [ current_datapoint_pos ] = temp_diff ;
                  current_datapoint_pos++ ;
                  saved_datapoints++ ;
                }

                else {
                  shiftDataPoints ( pulse_datapoints , NUM_DATAPOINTS , displayed_BPM ) ;
                  shiftDataPoints ( temp_datapoints , NUM_DATAPOINTS , temp_diff ) ;
                }
              }
              break ;

            case MENU_CHECKER :
              if ( flags & GRAPH_EN ) {
                flags &= ~GRAPH_EN ;
                ssd1306_clearScreen ( ) ;
                switch ( flags & SWITCH_GRAPH ) {
                  case 0 :
                    drawDataPoints ( pulse_datapoints , saved_datapoints ) ; break ;
                  case SWITCH_GRAPH :
                    drawDataPoints ( temp_datapoints , saved_datapoints ) ; break ;
                  default : break ;
                }
              }
              break ;
            default : break ;
          }
          break ;
        }
        else { printErrorMessage ( ) ; }
      default : break ;
    }
  }
}