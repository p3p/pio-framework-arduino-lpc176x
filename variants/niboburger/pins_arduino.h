/*
  NIBOburger library for ARDUINO
  License: BSD-License
  (c) 2013 by Nils Springob, nicai-systems
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>


#define TOTAL_PINS              32
#define TOTAL_ANALOG_PINS       8
#define FIRST_ANALOG_PIN        24

#define WLED                    17

// How many ports are on this device
#define WIRING_PORTS 4

/*************************************************************
 * Prototypes
 *************************************************************/

void boardInit(void);


/*************************************************************
 * Pin locations - constants
 *************************************************************/

// SPI port
const static uint8_t SS   = 20;
const static uint8_t MOSI = 21;
const static uint8_t MISO = 22;
const static uint8_t SCK  = 23;

// TWI port
const static uint8_t SCL  = 8;
const static uint8_t SDA  = 9;

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
const static uint8_t EI0 = 2;
const static uint8_t EI1 = 3;
const static uint8_t EI2 = 18;

// Hardware Serial port pins
const static uint8_t RX0 = 0;
const static uint8_t TX0 = 1;


// NIBO burger IO definitions

enum {
  // PORT D
  IO_RXD     =  0,
  IO_TXD     =  1,
  IO_ODO_L   =  2,
  IO_ODO_R   =  3,
  IO_PWM_L   =  4,
  IO_PWM_R   =  5,
  IO_DIR_L   =  6,
  IO_DIR_R   =  7,
 
  // PORT C
  IO_SCL     =  8,
  IO_SDA     =  9,
  IO_EN_FLL  = 10,
  IO_EN_FRR  = 11,
  IO_EN_FL   = 12,
  IO_EN_FR   = 13,
  IO_EN_BL   = 14,
  IO_EN_BR   = 15,
  
  // PORT B
  IO_EN_BC   = 16,
  IO_LED1    = 17,
  IO_LED2    = 18,
  IO_LED3    = 19,
  IO_LED4    = 20,
  IO_MOSI    = 21,
  IO_MISO    = 22,
  IO_SCK     = 23,
  
  // PORT A
  IO_AN_FL   = 24,
  IO_AN_FR   = 25,
  IO_AN_BL   = 26,
  IO_AN_BR   = 27,
  IO_AN_KEY  = 28,
  IO_AN_FLL  = 29,
  IO_AN_BC   = 30,
  IO_AN_FRR  = 31
};


/*************************************************************
 * Pin to register mapping macros
 *************************************************************/

#define digitalPinToPortReg(PIN) \
        ( ((PIN) >= 0  && (PIN) <= 7)  ? &PORTD : \
        ( ((PIN) >= 8  && (PIN) <= 15) ? &PORTC : \
        ( ((PIN) >= 16 && (PIN) <= 23) ? &PORTB : \
        ( ((PIN) >= 24 && (PIN) <= 31) ? &PORTA : NOT_A_REG))))

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
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &PORTA,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PIN,
	(uint16_t) &PINA,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
	// PORTLIST		
	// -------------------------------------------		
	PD	, // PD
	PD	, // PD
	PD	, // PD
	PD	, // PD
	PD	, // PD
	PD	, // PD
	PD	, // PD
	PD	, // PD
	PC	, // PC
	PC	, // PC
	PC	, // PC
	PC	, // PC
	PC	, // PC
	PC	, // PC
	PC	, // PC
	PC	, // PC
	PB	, // PB
	PB	, // PB
	PB	, // PB
	PB	, // PB
	PB	, // PB
	PB	, // PB
	PB	, // PB
	PB	, // PB
	PA	, // PA
	PA	, // PA
	PA	, // PA
	PA	, // PA
	PA	, // PA
	PA	, // PA
	PA	, // PA
	PA	, // PA
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
	// PIN IN PORT		
	// -------------------------------------------		
	_BV( 0 )	, // PD 0
	_BV( 1 )	, // PD 1
	_BV( 2 )	, // PD 2
	_BV( 3 )	, // PD 3
	_BV( 4 )	, // PD 4
	_BV( 5 )	, // PA 5
	_BV( 6 )	, // PA 6
	_BV( 7 )	, // PA 7
	_BV( 0 )	, // PA 0
	_BV( 1 )	, // PA 1
	_BV( 2 )	, // PA 2
	_BV( 3 )	, // PA 3
	_BV( 4 )	, // PA 4
	_BV( 5 )	, // PA 5
	_BV( 6 )	, // PA 6
	_BV( 7 )	, // PA 7
	_BV( 0 )	, // PA 0
	_BV( 1 )	, // PA 1
	_BV( 2 )	, // PA 2
	_BV( 3 )	, // PA 3
	_BV( 4 )	, // PA 4
	_BV( 5 )	, // PA 5
	_BV( 6 )	, // PA 6
	_BV( 7 )	, // PA 7
	_BV( 0 )	, // PA 0
	_BV( 1 )	, // PA 1
	_BV( 2 )	, // PA 2
	_BV( 3 )	, // PA 3
	_BV( 4 )	, // PA 4
	_BV( 5 )	, // PA 5
	_BV( 6 )	, // PA 6
	_BV( 7 )	, // PA 7
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
	// TIMERS		
	// -------------------------------------------		
	NOT_ON_TIMER	, // PD 0
	NOT_ON_TIMER	, // PD 1
	NOT_ON_TIMER	, // PD 2
	NOT_ON_TIMER	, // PD 3
	TIMER1B		, // PD 4
	TIMER1A		, // PA 5
	NOT_ON_TIMER	, // PA 6
	NOT_ON_TIMER	, // PH 4
	NOT_ON_TIMER	, // PH 5
	NOT_ON_TIMER	, // PH 6
	NOT_ON_TIMER	, // PB 4
	NOT_ON_TIMER	, // PB 5
	NOT_ON_TIMER	, // PB 6
	NOT_ON_TIMER	, // PB 7
	NOT_ON_TIMER	, // PJ 1
	NOT_ON_TIMER	, // PJ 0
	NOT_ON_TIMER	, // PH 1
	NOT_ON_TIMER	, // PH 0
	NOT_ON_TIMER	, // PD 3
	NOT_ON_TIMER	, // PD 2
	NOT_ON_TIMER	, // PD 1
	NOT_ON_TIMER	, // PD 0
	NOT_ON_TIMER	, // PA 0
	NOT_ON_TIMER	, // PA 1
	NOT_ON_TIMER	, // PA 2
	NOT_ON_TIMER	, // PA 3
	NOT_ON_TIMER	, // PA 4
	NOT_ON_TIMER	, // PA 5
	NOT_ON_TIMER	, // PA 6
	NOT_ON_TIMER	, // PA 7 ** 29 ** D29	
	NOT_ON_TIMER	, // PC 7 ** 30 ** D30	
	NOT_ON_TIMER	, // PC 6 ** 31 ** D31	
	NOT_ON_TIMER	, // PC 5 ** 32 ** D32	
	NOT_ON_TIMER	, // PC 4 ** 33 ** D33	
	NOT_ON_TIMER	, // PC 3 ** 34 ** D34	
	NOT_ON_TIMER	, // PC 2 ** 35 ** D35	
	NOT_ON_TIMER	, // PC 1 ** 36 ** D36	
	NOT_ON_TIMER	, // PC 0 ** 37 ** D37	
	NOT_ON_TIMER	, // PD 7 ** 38 ** D38	
	NOT_ON_TIMER	, // PG 2 ** 39 ** D39	
	NOT_ON_TIMER	, // PG 1 ** 40 ** D40	
	NOT_ON_TIMER	, // PG 0 ** 41 ** D41	
	NOT_ON_TIMER	, // PL 7 ** 42 ** D42	
	NOT_ON_TIMER	, // PL 6 ** 43 ** D43	
	NOT_ON_TIMER	, // PL 5 ** 44 ** D44	
	NOT_ON_TIMER	, // PL 4 ** 45 ** D45	
	NOT_ON_TIMER	, // PL 3 ** 46 ** D46	
	NOT_ON_TIMER	, // PL 2 ** 47 ** D47	
	NOT_ON_TIMER	, // PL 1 ** 48 ** D48	
	NOT_ON_TIMER	, // PL 0 ** 49 ** D49	
	NOT_ON_TIMER	, // PB 3 ** 50 ** SPI_MISO	
	NOT_ON_TIMER	, // PB 2 ** 51 ** SPI_MOSI	
	NOT_ON_TIMER	, // PB 1 ** 52 ** SPI_SCK	
	NOT_ON_TIMER	, // PB 0 ** 53 ** SPI_SS	
	NOT_ON_TIMER	, // PF 0 ** 54 ** A0	
	NOT_ON_TIMER	, // PF 1 ** 55 ** A1	
	NOT_ON_TIMER	, // PF 2 ** 56 ** A2	
	NOT_ON_TIMER	, // PF 3 ** 57 ** A3	
	NOT_ON_TIMER	, // PF 4 ** 58 ** A4	
	NOT_ON_TIMER	, // PF 5 ** 59 ** A5	
	NOT_ON_TIMER	, // PF 6 ** 60 ** A6	
	NOT_ON_TIMER	, // PF 7 ** 61 ** A7	
	NOT_ON_TIMER	, // PK 0 ** 62 ** A8	
	NOT_ON_TIMER	, // PK 1 ** 63 ** A9	
	NOT_ON_TIMER	, // PK 2 ** 64 ** A10	
	NOT_ON_TIMER	, // PK 3 ** 65 ** A11	
	NOT_ON_TIMER	, // PK 4 ** 66 ** A12	
	NOT_ON_TIMER	, // PK 5 ** 67 ** A13	
	NOT_ON_TIMER	, // PK 6 ** 68 ** A14	
	NOT_ON_TIMER	, // PK 7 ** 69 ** A15	
};

#endif


#endif
// BOARDDEFS_H
