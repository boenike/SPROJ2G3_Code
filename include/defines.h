#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <math.h>

#define Temp_Input           PC1
#define Bat_Input            PC2

#define Left_Button          PD2
#define Right_Button         PD3
#define Heart_Input          PD4

#define ADC_MAX_VAL          1023.0F
#define ADC_MIN_VAL          0.0F

#define OLED_WIDTH           128
#define OLED_HEIGHT          64

#define START_POS_X          5
#define MENU_GRAPH_Y         32
#define MENU_DATA_Y          16

#define CHAR_WIDTH           6
#define CHAR_LENGTH          8
#define MAX_CHARS_PER_NUM    5
#define MAX_CHARS_IN_ROW     20

#define VREF                 5.0F
#define BAT_MAX_VOLTAGE      4.2F
#define BAT_MIN_VOLTAGE      3.0F
#define BAT_THRESH_VOLTAGE   3.1F
#define BAT_MAX_READING      floor ( ( ADC_MAX_VAL * BAT_MAX_VOLTAGE ) / VREF )

#define BPM_LOWER_LIMIT      30.0F
#define BPM_UPPER_LIMIT      200.0F
#define PULSE_BPM_MULTIPLIER 60.0F

#define GRAPH_MAX_VERT_POS   9.0F
#define GRAPH_MIN_VERT_POS   61.0F

#define NUM_DATAPOINTS       20
#define MIN_DATAPOINTS       2

#define ARMPIT_MIN_TEMP      18.0F   // Minimum temperature reading - based on Op-amp circuit
#define ARMPIT_MAX_TEMP      45.0F   // Maximum temperature reading - based on Op-amp circuit

#define DECIMAL_PRECISION    2
#define NUM_BASE             10
#define MAX_BPM_SAMPLES      5
#define MAX_TEMP_SAMPLES     250

#define PRESCALER                1024.0F
#define PULSE_GATHER_INTERVAL    floor ( 1000.0F / ( BPM_LOWER_LIMIT / PULSE_BPM_MULTIPLIER ) )  // Timer1 delay in milliseconds
#define PULSE_PERIOD_UPPER_LIMIT floor ( 1000.0F / ( BPM_UPPER_LIMIT / PULSE_BPM_MULTIPLIER ) )
#define PULSE_CTR_UPPER_LIMIT    ( uint16_t ) ( ( 1.0F / ( BPM_UPPER_LIMIT / PULSE_BPM_MULTIPLIER ) ) / ( 1.0F / ( F_CPU / PRESCALER ) ) )
#define DELAY_TIME               15.0F // Timer2 delay in milliseconds
#define DELAY_MULTIPLIER         13    // Overall delay is (in milliseconds): DELAY_MULTIPLIER * DELAY_TIME
#define OCR1_VAL                 ( uint16_t ) floor ( ( ( F_CPU / PRESCALER ) * ( PULSE_GATHER_INTERVAL / 1000.0 ) ) - 1 )
#define OCR2_VAL                 ( uint8_t )  floor ( ( ( F_CPU / PRESCALER ) * ( DELAY_TIME / 1000.0 ) ) - 1 )

#define MENU_CHECKER           ( ( uint16_t ) ( 1 << 0  ) )
#define SELECT                 ( ( uint16_t ) ( 1 << 1  ) )
#define SWITCHED               ( ( uint16_t ) ( 1 << 2  ) )
#define BTN_BOUNCE_FLAG        ( ( uint16_t ) ( 1 << 3  ) )
#define FIRST_PULSE_MEASURED   ( ( uint16_t ) ( 1 << 4  ) )
#define CLEAR_EN               ( ( uint16_t ) ( 1 << 5  ) )
#define GRAPH_EN               ( ( uint16_t ) ( 1 << 6  ) )
#define SNAPSHOT_EN            ( ( uint16_t ) ( 1 << 7  ) )
#define SWITCH_GRAPH           ( ( uint16_t ) ( 1 << 8  ) )
#define BPM_EN                 ( ( uint16_t ) ( 1 << 9  ) )
#define MENU_SELECTOR          ( ( uint16_t ) ( 1 << 10 ) )
#define BAT_EN                 ( ( uint16_t ) ( 1 << 11 ) )

#ifdef __cplusplus
}
#endif