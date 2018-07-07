/*
  NIBO2 library for ARDUINO
  License: BSD-License
  (c) 2013 by Nils Springob, nicai-systems
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

#define TOTAL_PINS              53
#define TOTAL_ANALOG_PINS       8
#define FIRST_ANALOG_PIN        40

#define WLED                    16

// How many ports are on this device
#define WIRING_PORTS 7

/*************************************************************
 * Prototypes
 *************************************************************/

void boardInit(void);


/*************************************************************
 * Pin locations - constants
 *************************************************************/

// SPI port
const static uint8_t SS   = 8;
const static uint8_t MOSI = 10;
const static uint8_t MISO = 11;
const static uint8_t SCK  = 9;

// TWI port
const static uint8_t SCL  = 24;
const static uint8_t SDA  = 25;

// Analog pins
const static uint8_t A0 = 0;
const static uint8_t A1 = 1;
const static uint8_t A2 = 2;
const static uint8_t A3 = 3;
const static uint8_t A4 = 4;
const static uint8_t A5 = 5;
const static uint8_t A6 = 6;
const static uint8_t A7 = 7;

// External Interrupts
const static uint8_t EI0 = 24;
const static uint8_t EI1 = 25;
const static uint8_t EI2 = 26;
const static uint8_t EI3 = 27;

// Hardware Serial port pins
const static uint8_t RX0 = 32;
const static uint8_t TX0 = 33;
const static uint8_t RX1 = 26;
const static uint8_t TX1 = 27;


// NIBO 2 IO definitions

enum {
  /* PORT A */
  IO_LCD_D0     =  0,
  IO_LCD_D1     =  1,
  IO_LCD_D2     =  2,
  IO_LCD_D3     =  3,
  IO_LCD_D4     =  4,
  IO_LCD_D5     =  5,
  IO_LCD_D6     =  6,
  IO_LCD_D7     =  7,

  /* PORT B */
  IO_SS         =  8,
  IO_SCK        =  9,
  IO_MOSI       = 10,
  IO_MISO       = 11,
  IO_AUDIO      = 12,
  IO_LED_W_PWM  = 13,
  IO_LCD_PWM    = 14,
  IO_LED_RG_PWM = 15,

  /* PORT C */
  IO_LED_G0     = 16,
  IO_LED_G1     = 17,
  IO_LED_G2     = 18,
  IO_LED_G3     = 19,
  IO_LED_G4     = 20,
  IO_LED_G5     = 21,
  IO_LED_G6     = 22,
  IO_LED_G7     = 23,

  /* PORT D */
  IO_I2C_SCL    = 24,
  IO_I2C_SDA    = 25,
  IO_EXT_D      = 26,
  IO_EXT_C      = 27,
  IO_INPUT1     = 28,
  IO_EXT_B      = 29,
  IO_EXT_A      = 30,
  IO_RESET_IC2  = 31,
  
  /* PORT E */
  IO_LED_R0     = 32,
  IO_LED_R1     = 33,
  IO_LED_R2     = 34,
  IO_LED_R3     = 35,
  IO_LED_R4     = 36,
  IO_LED_R5     = 37,
  IO_LED_R6     = 38,
  IO_LED_R7     = 39,
  
  /* PORT F */
  IO_FLOOR0     = 40,
  IO_FLOOR1     = 41,
  IO_FLOOR2     = 42,
  IO_FLOOR3     = 43,
  IO_INPUT3     = 44,
  IO_INPUT2     = 45,
  IO_FLOOR_EN   = 46,
  IO_SUPPLY     = 47,
  
  /* PORT G */ 
  IO_LCD_CS1    = 48,
  IO_LCD_CS2    = 49,
  IO_LCD_EN     = 50,
  IO_LCD_RS     = 51,
  IO_LCD_RW     = 52
};

#define IO_RXD0 IO_LED_R0
#define IO_TXD0 IO_LED_R1
#define IO_RXD1 IO_EXT_D
#define IO_TXD1 IO_EXT_C



/*************************************************************
 * Pin to register mapping macros
 *************************************************************/

#define digitalPinToPortReg(PIN) \
        ( ((PIN) >=  0 && (PIN) <=  7) ? &PORTA : \
        ( ((PIN) >=  8 && (PIN) <= 15) ? &PORTB : \
        ( ((PIN) >= 16 && (PIN) <= 23) ? &PORTC : \
        ( ((PIN) >= 24 && (PIN) <= 31) ? &PORTD : \
        ( ((PIN) >= 32 && (PIN) <= 39) ? &PORTE : \
        ( ((PIN) >= 40 && (PIN) <= 47) ? &PORTF : \
        ( ((PIN) >= 48 && (PIN) <= 52) ? &PORTG : \
                                         NOT_A_REG))))

#define digitalPinToBit(P)       ((P) & 7)



/*************************************************************
 * Timer prescale factors
 *************************************************************/

#define TIMER0PRESCALEFACTOR 64


#ifdef ARDUINO_MAIN

const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &DDRA,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
	(uint16_t) &DDRE,
	(uint16_t) &DDRF,
	(uint16_t) &DDRG,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &PORTA,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
	(uint16_t) &PORTE,
	(uint16_t) &PORTF,
	(uint16_t) &PORTG,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PIN,
	(uint16_t) &PINA,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
	(uint16_t) &PINE,
	(uint16_t) &PINF,
	(uint16_t) &PING,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
	// PORTLIST		
	// -------------------------------------------		
	PA	, // PA 0
	PA	, // PA 1
	PA	, // PA 2
	PA	, // PA 3
	PA	, // PA 4
	PA	, // PA 5
	PA	, // PA 6
	PA	, // PA 7
	PB	, // PB 0
	PB	, // PB 1
	PB	, // PB 2
	PB	, // PB 3
	PB	, // PB 4
	PB	, // PB 5
	PB	, // PB 6
	PB	, // PB 7
	PC	, // PC 0
	PC	, // PC 1
	PC	, // PC 2
	PC	, // PC 3
	PC	, // PC 4
	PC	, // PC 5
	PC	, // PC 6
	PC	, // PC 7
	PD	, // PD 0
	PD	, // PD 1
	PD	, // PD 2
	PD	, // PD 3
	PD	, // PD 4
	PD	, // PD 5
	PD	, // PD 6
	PD	, // PD 7
	PE	, // PE 0
	PE	, // PE 1
	PE	, // PE 2
	PE	, // PE 3
	PE	, // PE 4
	PE	, // PE 5
	PE	, // PE 6
	PE	, // PE 7
	PF	, // PF 0
	PF	, // PF 1
	PF	, // PF 2
	PF	, // PF 3
	PF	, // PF 4
	PF	, // PF 5
	PF	, // PF 6
	PF	, // PF 7
	PG	, // PG 0
	PG	, // PG 1
	PG	, // PG 2
	PG	, // PG 3
	PG	, // PG 4
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
	// PIN IN PORT		
	// -------------------------------------------		
	_BV( 0 )	, // PA 0
	_BV( 1 )	, // PA 1
	_BV( 2 )	, // PA 2
	_BV( 3 )	, // PA 3
	_BV( 4 )	, // PA 4
	_BV( 5 )	, // PA 5
	_BV( 6 )	, // PA 6
	_BV( 7 )	, // PA 7
	_BV( 0 )	, // PB 0
	_BV( 1 )	, // PB 1
	_BV( 2 )	, // PB 2
	_BV( 3 )	, // PB 3
	_BV( 4 )	, // PB 4
	_BV( 5 )	, // PB 5
	_BV( 6 )	, // PB 6
	_BV( 7 )	, // PB 7
	_BV( 0 )	, // PC 0
	_BV( 1 )	, // PC 1
	_BV( 2 )	, // PC 2
	_BV( 3 )	, // PC 3
	_BV( 4 )	, // PC 4
	_BV( 5 )	, // PC 5
	_BV( 6 )	, // PC 6
	_BV( 7 )	, // PC 7
	_BV( 0 )	, // PD 0
	_BV( 1 )	, // PD 1
	_BV( 2 )	, // PD 2
	_BV( 3 )	, // PD 3
	_BV( 4 )	, // PD 4
	_BV( 5 )	, // PD 5
	_BV( 6 )	, // PD 6
	_BV( 7 )	, // PD 7
	_BV( 0 )	, // PE 0
	_BV( 1 )	, // PE 1
	_BV( 2 )	, // PE 2
	_BV( 3 )	, // PE 3
	_BV( 4 )	, // PE 4
	_BV( 5 )	, // PE 5
	_BV( 6 )	, // PE 6
	_BV( 7 )	, // PE 7
	_BV( 0 )	, // PF 0
	_BV( 1 )	, // PF 1
	_BV( 2 )	, // PF 2
	_BV( 3 )	, // PF 3
	_BV( 4 )	, // PF 4
	_BV( 5 )	, // PF 5
	_BV( 6 )	, // PF 6
	_BV( 7 )	, // PF 7
	_BV( 0 )	, // PG 0
	_BV( 1 )	, // PG 1
	_BV( 2 )	, // PG 2
	_BV( 3 )	, // PG 3
	_BV( 4 )	, // PG 4
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
	/* PORT A */
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	
	/* PORT B */
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	TIMER0A,
	TIMER1A,
	TIMER1B,
	TIMER1C,
	
	/* PORT C */
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	
	/* PORT D */
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	
	/* PORT E */
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	TIMER3A,
	TIMER3B,
	TIMER3C,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	
	/* PORT F */
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	
	/* PORT G */
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER
};

#endif


#endif
// BOARDDEFS_H
