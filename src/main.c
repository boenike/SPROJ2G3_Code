/* Title: Semester Project 2 Group 3 - Main Code
 * Institution: University of Southern Denmark (SDU)
 * Campus: Sonderborg
 * File: main.c
 * Author: Bence Toth
 * Date: 21/05/2024
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
#warning "CPU clock was not defined! Set to 16 MHz"
#define F_CPU 16000000UL
// Clock frequency of ATmega328P is 16 MHz -> Only define outside of PlatformIO
#endif

#ifdef ARDUINO
#undef ARDUINO
#endif

#define Onboard_LED PB5
#define Heart_Enable PB1

#define Temp_Input PC1
#define Bat_Input PC2

#define Left_Button PD2
#define Right_Button PD3
#define Heart_Input PD4

#define OLED_WIDTH 128
#define OLED_HEIGHT 64

#define START_POS_X 5
#define MENU_GRAPH_Y 32
#define MENU_DATA_Y 16
#define MENU_BAT_X 5
#define MENU_BAT_Y 48

#define CHAR_WIDTH 6
#define CHAR_LENGTH 8
#define MAX_CHARS_PER_NUM 5
#define MAX_CHARS_IN_ROW 20

#define BAT_MAX_VOLTAGE 4.2F
#define BAT_MIN_VOLTAGE 3.1F

#define BPM_LOWER_LIMIT 30.0F
#define BPM_UPPER_LIMIT 200.0F

#define GRAPH_MAX_VERT_POS 9.0F
#define GRAPH_MIN_VERT_POS 61.0F

#define NUM_DATAPOINTS 20
#define MIN_DATAPOINTS 2

#define ADC_MAX_VAL 1023.0F
#define ADC_MIN_VAL 0.0F

#define ARMPIT_MIN_TEMP 17.0F
#define ARMPIT_MAX_TEMP 42.0F

#define DECIMAL_PRECISION 2
#define NUM_BASE 10
#define MAX_SAMPLES 4

#define PRESCALER 1024.0F
#define PULSE_GATHER_INTERVAL floor ( 1000.0F / ( BPM_LOWER_LIMIT / 60.0F ) )  // Timer1 delay in milliseconds
#define DELAY_TIME 15.0F               // Timer2 delay in milliseconds
#define DELAY_MULTIPLIER 13            // Overall delay is (in milliseconds): DELAY_MULTIPLIER * DELAY_TIME
#define OCR1_VAL (uint16_t) floor ( ( (F_CPU/PRESCALER) * ( PULSE_GATHER_INTERVAL / 1000.0 ) ) - 1 )
#define OCR2_VAL (uint8_t)  floor ( ( (F_CPU/PRESCALER) * ( DELAY_TIME / 1000.0 ) ) - 1 )

typedef enum { MENU , DATA } menu_style_t ;
volatile menu_style_t menu_selector = MENU ;
volatile uint8_t menu_checker = 0 , select = 0 , switched = 0 , btn_bounce_flag = 1 , timeout_checker = 0 , first_pulse_measured = 0 ;
volatile uint8_t clear_en = 0 , graph_en = 1 , snapshot_en = 0 , clear_graph_en = 0 , BPM_en = 0 , interval_check = 0 ;
volatile uint16_t pulse_duration = 0 ;
double BPM = 0.0 , displayed_BPM = 0.0 ;
double datapoints [ NUM_DATAPOINTS ] ;
char text [ MAX_CHARS_IN_ROW ] ;
const char indicator [ ] = "->" ;
uint8_t current_datapoint_pos = 0 , saved_datapoints = 0 , no_data_flag = 0 ;

ISR ( TIMER2_COMPA_vect )  {
  cli ( ) ;
  timeout_checker++ ;
  if ( timeout_checker == DELAY_MULTIPLIER ) {
    btn_bounce_flag = 1 ; // Enable button flag
    timeout_checker = 0 ; // Reset timeout checking
    TCCR2B = 0x00 ;       // Stop Timer2
  }
  sei ( ) ;
}

ISR ( TIMER1_COMPA_vect ) {
  cli ( ) ;
  TCNT0 = 0x00 ;        // Reset interval after an overflow
  interval_check = 0 ; 
  TCNT1 = 0x0000 ;
  TCCR1B &= ~( ( 1 << CS12 ) | ( 1 << CS10 ) ) ;   // Disable Timer1
  sei ( ) ;
}

ISR ( TIMER0_COMPA_vect ) {
  cli ( ) ;
  interval_check++ ;
  switch ( interval_check ) {
    case 1 :
      // Start Timer1 with prescaler set to 1024
      TCNT1 = 0x0000 ;        // Reset Timer1 value
      TCCR1B |= ( 1 << CS12 ) | ( 1 << CS10 ) ;
      break ;
    case 2 :
      pulse_duration = TCNT1 ;
      interval_check = 0 ;
      TCNT1 = 0x0000 ;
      TCCR1B &= ~( ( 1 << CS12 ) | ( 1 << CS10 ) ) ;   // Disable Timer1
      BPM_en = 1 ;
      first_pulse_measured = 1 ;
      break ;
    default :
      break ;
  }
  sei ( ) ;
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

  // At first, stop pulse count
  PORTB |= ( 1 << Heart_Enable ) ;  // Turn off Enable pin
  pulse_duration = 0 ;
}

uint8_t clearDataPoints ( double data [ ] , uint8_t data_length ) {
  uint8_t ctr ;
  for ( ctr = 0 ; ctr < data_length ; ctr++ ) {
    data [ ctr ] = 0.0 ;
  }
  return 1 ;
}

uint8_t allDatapointsEqual ( double data [ ] , uint8_t stop_point ) {
  uint8_t ctr ;
  double reference = data [ 0 ] ;
  for ( ctr = 1 ; ctr < stop_point ; ctr++ ) {
    if ( data [ ctr ] != reference ) return 0 ;
  }
  return 1 ;
}

double convertInterval ( double x , double in_min , double in_max , double out_min , double out_max ) {
  return ( x - in_min ) * ( out_max - out_min ) / ( in_max - in_min ) + out_min ;
}

void drawBoundary ( menu_style_t MENU_TYPE ) {
  ssd1306_drawHLine ( 0 , OLED_HEIGHT - 1 , OLED_WIDTH - 1 ) ; // Bottom
  ssd1306_drawHLine ( 0 , 0 , 45 ) ; // Top 1
  ssd1306_drawHLine ( 86 , 0 , OLED_WIDTH - 1 ) ; // Top 2
  ssd1306_drawVLine ( 0 , 0 , OLED_HEIGHT - 1 ) ; // Left
  ssd1306_drawVLine ( OLED_WIDTH - 1 , 0 , OLED_HEIGHT - 1 ) ; // Right

  switch ( MENU_TYPE ) {
    case MENU :
      ssd1306_printFixed ( 45 , 0 , " Menu " , STYLE_NORMAL ) ;
      break ;
    case DATA :
      ssd1306_printFixed ( 45 , 0 , " Data " , STYLE_NORMAL ) ;
      break ;
    default :
      break ;
  }
}

void printErrorMessage ( void ) {
  ssd1306_printFixed ( 5 , 30 , "LM75 inaccessible!" , STYLE_NORMAL ) ;
  ssd1306_printFixed ( 20 , 40 , "Please insert" , STYLE_NORMAL ) ;
  ssd1306_printFixed ( 5 , 50 , "external temp module" , STYLE_NORMAL ) ;
}

void findSmallestAndBiggest ( double data [ ] , double *min , double *max , uint16_t stop_pos ) {
  uint8_t ctr ;
  double smallest = data [ 0 ] , biggest = data [ 0 ] ;
  for ( ctr = 0 ; ctr < stop_pos ; ctr++ ) {
    if ( data [ ctr ] < smallest ) smallest = data [ ctr ] ;
    if ( data [ ctr ] > biggest ) biggest = data [ ctr ] ;
  }
  * min = smallest ;
  * max = biggest ;
}

void drawDataPoints ( double data [ ] , uint8_t stop_position ) {
  uint8_t halver ;

  if ( stop_position <= MIN_DATAPOINTS ) {
    char MIN_DATAPOINTS_BUF [ 2 ] ;
    itoa ( MIN_DATAPOINTS , MIN_DATAPOINTS_BUF , NUM_BASE ) ;
    ssd1306_printFixed ( 2 , 8 , "Minimum of" , STYLE_NORMAL ) ;
    ssd1306_printFixed ( 10 , 16 , MIN_DATAPOINTS_BUF , STYLE_BOLD ) ;
    ssd1306_printFixed ( 25 , 16 , "datapoints" , STYLE_NORMAL ) ;
    ssd1306_printFixed ( 15 , 24 , "have to be given!" , STYLE_NORMAL ) ;
    return ;
  }

  switch ( allDatapointsEqual ( data , stop_position ) ) {
    case 0 :
      halver = 0 ;
      break ;
    case 1 :
      halver = 30 ;
      break ;
    default :
      break ;
  }

  uint8_t ctr , x1 = 3 , x2 , y1 , y2 , offset ;
  double MIN , MAX ;
  
  findSmallestAndBiggest ( datapoints , &MIN , &MAX , saved_datapoints ) ;

  offset = (uint8_t) floor ( (double) OLED_WIDTH / stop_position ) ;
  x2 = offset ;
  y1 = (uint8_t) convertInterval ( data [ 0 ] , MIN , MAX , GRAPH_MIN_VERT_POS , GRAPH_MAX_VERT_POS ) - halver ;

  for ( ctr = 0 ; ctr < stop_position ; ctr++ ) {
    y2 = (uint8_t) convertInterval ( data [ ctr ] , MIN , MAX , GRAPH_MIN_VERT_POS , GRAPH_MAX_VERT_POS ) - halver ;
    ssd1306_drawLine ( x1 , y1 , x2 , y2 ) ;
    x1 = x2 ;
    y1 = y2 ;
    x2 += offset ;
  }
  snprintf ( text , MAX_CHARS_IN_ROW , "Min %.0f Max %.0f" , MIN , MAX ) ;
  ssd1306_printFixed ( 0 , 0 , text , STYLE_NORMAL ) ;
}

void shiftDataPoints ( double data [ ] , uint8_t data_length , double new_data ) {
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
  //uint8_t samples = 0 ;
  double temp_analog , temp_digital /*, avg_BPM = 0.0*/ ;
  
  temp_digital = get_temperature ( ) ;
  potval = read_ADC ( Temp_Input ) ;
  temp_analog = convertInterval ( ( ADC_MAX_VAL - (double) potval ) , ADC_MIN_VAL , ADC_MAX_VAL , ARMPIT_MIN_TEMP , ARMPIT_MAX_TEMP ) ;

  snprintf ( text , MAX_CHARS_IN_ROW , "Pulse [BPM]:" ) ;
  ssd1306_printFixed ( 3 , 24 , text , STYLE_NORMAL ) ;

  snprintf ( text , MAX_CHARS_IN_ROW , "Armpit  [C]: %.2f " , temp_analog ) ;
  ssd1306_printFixed ( 3 , 32 , text , STYLE_NORMAL ) ;

  snprintf ( text , MAX_CHARS_IN_ROW , "Finger  [C]: %.2f " , temp_digital ) ;
  ssd1306_printFixed ( 3 , 40 , text , STYLE_NORMAL ) ;

  if ( BPM_en ) {
    BPM_en = 0 ;
    BPM = 120.0 / convertInterval ( (double) pulse_duration , 0.0 , (double) OCR1_VAL , 0.0 , PULSE_GATHER_INTERVAL / 1000.0 ) ;

    if ( BPM <= BPM_UPPER_LIMIT && BPM >= BPM_LOWER_LIMIT ) {
      displayed_BPM = BPM ;
      snprintf ( text , MAX_CHARS_IN_ROW , "Pulse [BPM]: %.0f   " , displayed_BPM ) ;
      ssd1306_printFixed ( 3 , 24 , text , STYLE_NORMAL ) ;
      /*samples++ ;
      avg_BPM += BPM ;
      if ( samples >= MAX_SAMPLES ) { // Taking the average to smooth out the BPM reading
        avg_BPM /= samples ;
        snprintf ( text , MAX_CHARS_IN_ROW , "Pulse [BPM]: %.0f   " , avg_BPM ) ;
        ssd1306_printFixed ( 3 , 24 , text , STYLE_NORMAL ) ;
        displayed_BPM = avg_BPM ;
        avg_BPM = 0.0 ;
        samples = 0 ;
      }*/
    }
  }
}

ISR ( INT0_vect ) {
  cli ( ) ;
  if ( btn_bounce_flag ) {

    btn_bounce_flag = 0 ; // Disable button flag
    // Set prescaler to 1024 and start Timer2
    TCCR2B = ( 1 << CS22 ) | ( 1 << CS21 ) | ( 1 << CS20 ) ;

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
          case 1 :
            graph_en = 1 ;
            clear_graph_en = 1 ;
            break ;
          default :
            break ;
        }
        break ;
      default :
        break ;
    }
  }
  sei ( ) ;
}

ISR ( INT1_vect ) {
  cli ( ) ;
  if ( btn_bounce_flag ) {

    btn_bounce_flag = 0 ; // Disable button flag
    // Set prescaler to 1024 and start Timer2
    TCCR2B = ( 1 << CS22 ) | ( 1 << CS21 ) | ( 1 << CS20 ) ;

    select = !select ;
    clear_en = 1 ;

    switch ( select ) {

      case 0 :
        switched = 0 ;
        first_pulse_measured = 0 ;
        // Stop pulse count
        //PORTB |= ( 1 << Heart_Enable ) ;  // Turn off Enable pin
        TCCR0B = 0x00 ;                   // Disable Counter0
        pulse_duration = 0 ;
        menu_selector = MENU ;
        break ;

      case 1 :
        switch ( menu_checker ) {
          case 0 :
            menu_selector = DATA ;
            // Start pulse count
            //PORTB &= ~( 1 << Heart_Enable ) ;  // Turn on Enable pin
            // External Clock Source on PD4, triggered at a rising edge
            // Start Counter0 with prescalers set to 1024
            TCNT0 = 0x00 ;    // Reset Counter0 value
            TCCR0B = ( 1 << CS02 ) | ( 1 << CS01 ) | ( 1 << CS00 ) ;
            break ;
          case 1 :
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
  sei ( ) ;
}

int main ( void ) {
  EFontStyle real_style , graph_style ;
  uint8_t menu_data_x , menu_graph_x , indicator_pos_y , LM75_accessed ;
  /*uint16_t bat_ADC_reading ;
  double bat_voltage ;
  char bat_volt_str [ MAX_CHARS_PER_NUM ] ;*/

  init_Pins ( ) ; 
  setup_Timers ( ) ;
  init_External_Interrupts ( ) ;
  init_ADC ( ) ;

  ssd1306_128x64_i2c_init ( ) ;
  ssd1306_clearScreen ( ) ;
  ssd1306_positiveMode ( ) ;
  ssd1306_setFixedFont ( ssd1306xled_font6x8 ) ;
  drawBoundary ( MENU ) ;

  //i2c_init ( ) ;
  LM75_accessed = lm75_init ( ) ;

  no_data_flag = clearDataPoints ( datapoints , NUM_DATAPOINTS ) ;

  while ( 1 ) {

    if ( clear_en && LM75_accessed ) {
      clear_en = 0 ;
      ssd1306_clearScreen ( ) ;
      if ( !switched || !menu_checker ) { drawBoundary ( menu_selector ) ; }
    }

    switch ( select ) {

      case 0 :
        /*bat_ADC_reading = read_ADC ( Bat_Input ) ;
        bat_voltage = ( bat_ADC_reading * BAT_MAX_VOLTAGE ) / ADC_MAX_VAL ;
        dtostrf ( bat_voltage , MAX_CHARS_PER_NUM , DECIMAL_PRECISION , bat_volt_str ) ;
        ssd1306_printFixed ( MENU_BAT_X + ( 14 * CHAR_WIDTH ) , MENU_BAT_Y , bat_volt_str , STYLE_NORMAL ) ;*/

        if ( !switched ) {
          switched = 1 ;
          
          //ssd1306_printFixed ( MENU_BAT_X , MENU_BAT_Y , "Bat. volt.: " , STYLE_NORMAL ) ;

          if ( LM75_accessed ) {
            switch ( menu_checker ) {
              case 0 :
                real_style = STYLE_BOLD ;
                graph_style = STYLE_ITALIC ;
                indicator_pos_y = MENU_DATA_Y ;
                menu_data_x = START_POS_X + CHAR_WIDTH * sizeof ( indicator - 1 ) ;
                menu_graph_x = START_POS_X ;
                break ;
              case 1 :
                real_style = STYLE_ITALIC ;
                graph_style = STYLE_BOLD ;
                indicator_pos_y = MENU_GRAPH_Y ;
                menu_graph_x = START_POS_X + CHAR_WIDTH * sizeof ( indicator - 1 ) ;
                menu_data_x = START_POS_X ;
                break ;
              default :
                break ;
            }
          
            ssd1306_printFixed ( START_POS_X , indicator_pos_y , indicator , STYLE_NORMAL ) ;
            ssd1306_printFixed ( menu_data_x , MENU_DATA_Y , " Real-time data  " , real_style ) ;
            ssd1306_printFixed ( menu_graph_x , MENU_GRAPH_Y , " Pulse graph  " , graph_style ) ;
          }
          else { printErrorMessage ( ) ; }
        }
        break ;

      case 1 :
        if ( LM75_accessed ) {
          switch ( menu_checker ) {
            case 0 :

              realTimeData ( ) ;

              if ( snapshot_en && first_pulse_measured ) {
                snapshot_en = 0 ;
                no_data_flag = 0 ;
                if ( saved_datapoints < NUM_DATAPOINTS ) {
                  datapoints [ current_datapoint_pos ] = displayed_BPM ;
                  current_datapoint_pos++ ;
                  saved_datapoints++ ;
                }

                else shiftDataPoints ( datapoints , NUM_DATAPOINTS , displayed_BPM ) ;
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
                    ssd1306_printFixed ( 8 , 32 , "NO DATA TO DISPLAY" , STYLE_BOLD ) ;
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
        }
        else { printErrorMessage ( ) ; }
      default :
        break ;
    }
  }
}