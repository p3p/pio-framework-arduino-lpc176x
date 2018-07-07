#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

// ATMEL ATMEGA1284P
// (Notice that SODAQ Mbili is equiped with VQFN44 package. The layout below is
// just a convenience to see the port names and their usage.)
//                                          +---\/---+
//                             (D 23) PB0  1|        |40  PA0 (AI 0 / D27) -- [A3]
//      [D14/DIGITAL2] --      (D 14) PB1  2|        |39  PA1 (AI 1 / D26) -- [A2]
//      [D15/DIGITAL1] -- INT2 (D 15) PB2  3|        |38  PA2 (AI 2 / D25) -- [A1]
//       [LED_RED] --      PWM (D 16) PB3  4|        |37  PA3 (AI 3 / D24) -- [A0]
//        [D10/SS] --   PWM/SS (D 10) PB4  5|        |36  PA4 (AI 4 / D30) -- [A6/BATVOLTPIN]
//      [D11/MOSI] --     MOSI (D 11) PB5  6|        |35  PA5 (AI 5 / D28) -- [A4] (Schema A6)
//      [D12/MISO] -- PWM/MISO (D 12) PB6  7|        |34  PA6 (AI 6 / D39) -- [A5] (Schema A7)
//       [D13/SCK] --  PWM/SCK (D 13) PB7  8|        |33  PA7 (AI 7 / D31) -- [A7/RTC_INTERRUPT]
//                                    RST  9|        |32  AREF
//                                    VCC 10|        |31  GND
//                                    GND 11|        |30  AVCC
//                                  XTAL2 12|        |29  PC7 (D  9)     -- [D9]
//                                  XTAL1 13|        |28  PC6 (D  8)     -- [D8]
//       [D0/RX0] -- RX0      (D  0)  PD0 14|        |27  PC5 (D 17) TDI -- [LED_GREEN]
//       [D1/TX0] -- TX0      (D  1)  PD1 15|        |26  PC4 (D 18) TDO -- [LED_BLUE]
//  [RX1/LoRa_TX] -- RX1/INT0 (D 19)  PD2 16|        |25  PC3 (D  7) TMS -- [D7]
//  [TX1/LoRa_RX] -- TX1/INT1 (D 20)  PD3 17|        |24  PC2 (D  6) TCK -- [D6]
//           [D2] -- PWM      (D  2)  PD4 18|        |23  PC1 (D 22) SDA -- [SDA]
//           [D3] -- PWM      (D  3)  PD5 19|        |22  PC0 (D 21) SCL -- [SCL]
//           [D4] -- PWM      (D  4)  PD6 20|        |21  PD7 (D  5) PWM -- [D5]
//                                          +--------+
//

/*
 * Arduino digital pin numbers:
 * 0..32
 * PD0..PD7, PB0..PB7, PC0..PC7, PA0..PA7
 */

#define NUM_DIGITAL_PINS            32
#define NUM_ANALOG_INPUTS           8

#define analogInputToDigitalPin(p)  ((p < NUM_ANALOG_INPUTS) ? (p) + 24 : -1)
#define analogPinToChannel(p)       (((p) == A0) ? 3 : ((p) == A1) ? 2 : ((p) == A2) ? 1 : ((p) == A3) ? 0 : ((p) == A4) ? 5 : ((p) == A5) ? 6 : ((p) == A6) ? 4 : ((p) == A7) ? 7 : 0);

#define digitalPinHasPWM(p)         ((p) == 2 || (p) == 3 || (p) == 4 || (p) == 5 || (p) == 10 || (p) == 12 || (p) == 13 || (p) == 16)

static const uint8_t SS   = 10;
static const uint8_t MOSI = 11;
static const uint8_t MISO = 12;
static const uint8_t SCK  = 13;

static const uint8_t SDA = 22;
static const uint8_t SCL = 21;

static const uint8_t LED_RED   = 16;
static const uint8_t LED_GREEN = 17;
static const uint8_t LED_BLUE  = 18;

static const uint8_t BAT_VOLT   = 30;           // A6
#define BATVOLT_R1                47            // in fact 4.7M
#define BATVOLT_R2               100            // in fact 10M

static const uint8_t RTC_INTERRUPT = 31;        // A7

static const uint8_t A0 = 24;
static const uint8_t A1 = 25;
static const uint8_t A2 = 26;
static const uint8_t A3 = 27;
static const uint8_t A4 = 28;
static const uint8_t A5 = 29;
static const uint8_t A6 = 30;
static const uint8_t A7 = 31;

/*
   PCINT31-24: D7-0   : bit 3
   PCINT15-8:  D15-8  : bit 1
   PCINT23-16: D23-16 : bit 2
   PCINT7-0:   D31-24 : bit 0 (also A0..A7)
*/

#define digitalPinToPCICR(p)    (((p) >= 0 && (p) < NUM_DIGITAL_PINS) ? (&PCICR) : ((uint8_t *)0))
#define digitalPinToPCICRbit(p) (pgm_read_byte(&(digital_pin_to_port_PGM[p])) - 1) // PCICRbit = Port Index (PA, PB, PC, PD) - 1

#define digitalPinToPCMSK(p)    (pgm_read_word(&(port_to_PCMSK_PGM[pgm_read_byte(&(digital_pin_to_port_PGM[p]))])))
#define digitalPinToPCMSKbit(p) (pgm_read_byte(&(digital_pin_to_port_pin_PGM[p])))

#ifdef ARDUINO_MAIN

// These are used as index in port_to_XYZ arrays, right?
#define PA 1
#define PB 2
#define PC 3
#define PD 4

// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] =
{
  NOT_A_PORT,
  (uint16_t) &DDRA,
  (uint16_t) &DDRB,
  (uint16_t) &DDRC,
  (uint16_t) &DDRD,
};

const uint16_t PROGMEM port_to_output_PGM[] =
{
  NOT_A_PORT,
  (uint16_t) &PORTA,
  (uint16_t) &PORTB,
  (uint16_t) &PORTC,
  (uint16_t) &PORTD,
};

const uint16_t PROGMEM port_to_input_PGM[] =
{
  NOT_A_PORT,
  (uint16_t) &PINA,
  (uint16_t) &PINB,
  (uint16_t) &PINC,
  (uint16_t) &PIND,
};

const uint16_t PROGMEM port_to_PCMSK_PGM[] =
{
  NOT_A_PORT,
  (uint16_t) &PCMSK0,
  (uint16_t) &PCMSK1,
  (uint16_t) &PCMSK2,
  (uint16_t) &PCMSK3,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] =
{
  PD, /* 0 */
  PD,
  PD,
  PD,
  PD,
  PD,
  PC,
  PC,
  PC, /* 8 */
  PC,
  PB,
  PB,
  PB,
  PB,
  PB,
  PB,
  PB, /* 16 */
  PC,
  PC,
  PD,
  PD,
  PC,
  PC,
  PB,
  PA, /* 24 */
  PA,
  PA,
  PA,
  PA,
  PA,
  PA,
  PA  /* 31 */
};

const uint8_t PROGMEM digital_pin_to_port_pin_PGM[] =
{
  0, /* 0 */
  1,
  4,
  5,
  6,
  7,
  2,
  3,
  6, /* 8 */
  7,
  4,
  5,
  6,
  7,
  1,
  2,
  3, /* 16 */
  5,
  4,
  2,
  3,
  0,
  1,
  0,
  3, /* 24 */
  2,
  1,
  0,
  5,
  6,
  4,
  7
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] =
{
  _BV(0), /* 0 */
  _BV(1),
  _BV(4),
  _BV(5),
  _BV(6),
  _BV(7),
  _BV(2),
  _BV(3),
  _BV(6), /* 8 */
  _BV(7),
  _BV(4),
  _BV(5),
  _BV(6),
  _BV(7),
  _BV(1),
  _BV(2),
  _BV(3), /* 16 */
  _BV(5),
  _BV(4),
  _BV(2),
  _BV(3),
  _BV(0),
  _BV(1),
  _BV(0),
  _BV(3), /* 24 */
  _BV(2),
  _BV(1),
  _BV(0),
  _BV(5),
  _BV(6),
  _BV(4),
  _BV(7)
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] =
{
  NOT_ON_TIMER,   /* 0  - PD0 */
  NOT_ON_TIMER,   /* 1  - PD1 */
  TIMER1B,        /* 4  - PD4 */
  TIMER1A,        /* 5  - PD5 */
  TIMER2B,        /* 6  - PD6 */
  TIMER2A,        /* 7  - PD7 */
  NOT_ON_TIMER,   /* 18 - PC2 */
  NOT_ON_TIMER,   /* 19 - PC3 */
  NOT_ON_TIMER,   /* 22 - PC6 */
  NOT_ON_TIMER,   /* 23 - PC7 */
  TIMER0B,        /* 12 - PB4 */
  NOT_ON_TIMER,   /* 13 - PB5 */
  TIMER3A,        /* 14 - PB6 */
  TIMER3B,        /* 15 - PB7 */
  NOT_ON_TIMER,   /* 9  - PB1 */
  NOT_ON_TIMER,   /* 10 - PB2 */
  TIMER0A,        /* 11 - PB3 */
  NOT_ON_TIMER,   /* 21 - PC5 */
  NOT_ON_TIMER,   /* 20 - PC4 */
  NOT_ON_TIMER,   /* 2  - PD2 */
  NOT_ON_TIMER,   /* 3  - PD3 */
  NOT_ON_TIMER,   /* 16 - PC0 */
  NOT_ON_TIMER,   /* 17 - PC1 */
  NOT_ON_TIMER,   /* 8  - PB0 */
  NOT_ON_TIMER,   /* 27 - PA3 */
  NOT_ON_TIMER,   /* 26 - PA2 */
  NOT_ON_TIMER,   /* 25 - PA1 */
  NOT_ON_TIMER,   /* 24 - PA0 */
  NOT_ON_TIMER,   /* 29 - PA5 */
  NOT_ON_TIMER,   /* 30 - PA6 */
  NOT_ON_TIMER,   /* 28 - PA4 */
  NOT_ON_TIMER    /* 31 - PA7 */
};

#endif // ARDUINO_MAIN

#endif // Pins_Arduino_h
