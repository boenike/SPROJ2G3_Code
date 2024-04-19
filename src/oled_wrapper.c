#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "ssd1306.h"
#include "oled_wrapper.h"

void printNumber ( uint8_t x_pos , uint8_t y_pos , numtype_t NUMTYPE , number_t *NUMBER , enum E_Font font ) {
  char NUM_BUFFER [ NUM_LEN ] ;

  switch ( NUMTYPE ) {
   case INTEGER : itoa ( NUMBER->intgr , NUM_BUFFER , RADIX ) ; break ;
   case DOUBLE : dtostrf ( NUMBER->dbl , NUM_LEN , DEC_PRECISION , NUM_BUFFER ) ; break ;
   default : break ;
  }

  SSD1306_SetPosition ( x_pos , y_pos ) ;
  SSD1306_DrawString ( NUM_BUFFER , font ) ;
}

void printString ( char *text , uint8_t x_pos , uint8_t y_pos , enum E_Font font ) {
  SSD1306_SetPosition ( x_pos , y_pos ) ;
  SSD1306_DrawString ( text , font ) ;
}

void addChars ( char *first , char *second , char *return_str ) {
  uint8_t ctr , idx = strlen ( first ) , offset = strlen ( second ) , remainder = strlen ( return_str ) - idx - offset ;
  
  // Copy the contents of the two char arrays into the buffer
  memcpy ( return_str , first , idx ) ;
	memcpy ( &return_str [ idx ] , second , offset ) ;       // Add the numerical value to the char array

  if ( remainder > 0 ) {
    char blanks [ remainder ] ;
    for ( ctr = 0 ; ctr < remainder ; ctr++ ) {
      blanks [ ctr ] = ' ' ;
    }
    memcpy ( &return_str [ idx + offset ] , blanks , remainder ) ;   // Add some blank space characters at the end
  }
}

void printStringWithNumber ( char *text , uint8_t x_pos , uint8_t y_pos , numtype_t NUMTYPE , number_t *NUMBER , enum E_Font font , numplacement_t placement ) {
  // Use when displaying characters as well as numerical values

  char TEXT_BUFFER [ strlen ( text ) + NUM_LEN ] , NUM_BUFFER [ NUM_LEN ] ;

  switch ( NUMTYPE ) {
   case INTEGER : itoa ( NUMBER->intgr , NUM_BUFFER , RADIX ) ; break ;
   case DOUBLE : dtostrf ( NUMBER->dbl , NUM_LEN , DEC_PRECISION , NUM_BUFFER ) ; break ;
   default : break ;
  }

  switch ( placement ) {
    case BEFORE : addChars ( NUM_BUFFER , text , TEXT_BUFFER ) ; break ;
    case AFTER : addChars ( text , NUM_BUFFER , TEXT_BUFFER ) ; break ;
    default : break ;
  }
  
  SSD1306_SetPosition ( x_pos , y_pos ) ;
  SSD1306_DrawString ( TEXT_BUFFER , font ) ;
}