#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#ifndef OLED_WRAPPER_H_INCLUDED

#define OLED_WRAPPER_H_INCLUDED
#define ROW_LEN 21
#define NUM_LEN 6
#define DEC_PRECISION 2
#define RADIX 10

// Used for structurizing numbers for printing to the OLED screen
typedef enum { INTEGER , DOUBLE } numtype_t ;
typedef enum { BEFORE , AFTER } numplacement_t ;
typedef union { double dbl ; int32_t intgr ; } number_t ;

// Function prototypes
void printNumber ( uint8_t x_pos , uint8_t y_pos , numtype_t NUMTYPE , number_t *NUMBER , enum E_Font font ) ;
void printString ( char *text , uint8_t x_pos , uint8_t y_pos , enum E_Font font ) ;
void addChars ( char *first , char *second , char *return_str ) ;
void printStringWithNumber ( char *text , uint8_t x_pos , uint8_t y_pos , numtype_t NUMTYPE , number_t *NUMBER , enum E_Font font , numplacement_t placement ) ;

#endif