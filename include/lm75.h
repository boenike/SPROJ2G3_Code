#ifdef __cplusplus
 extern "C" {
#endif

#ifndef LM75_H_INCLUDED
#define LM75_H_INCLUDED
#include <stdint.h>

// LM75 I2C address: (A2 = A1 = A0 = GND = 0V)
#define LM75_ADR 0x90

// Function headers
double get_temperature ( void ) ;
//void lm75_init(void);
uint8_t lm75_init ( void ) ;

#endif
#ifdef __cplusplus
}
#endif