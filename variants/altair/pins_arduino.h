/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

  $Id: wiring.h 249 2007-02-03 16:52:51Z mellis $
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

#define NUM_DIGITAL_PINS            35
#define NUM_ANALOG_INPUTS           8
#define analogPinToChannel(P)  (  (pin >= 20) ? pin - 20 : pin )
#define analogInputToDigitalPin(p)  ((p < 8) ? (p) + 20 : -1)
#define digitalPinHasPWM(p)         ((p) == 9 || (p) == 10 || (p) == 11 || (p) == 12 || (p) == 13 || (p) == 14 || (p) == 15 || (p) == 19)

#define SS              8
#define SCK             28
#define MOSI            29
#define MISO            30

#define SCL             0
#define SDA             1

#define RX1             2
#define TX1             3

#define RX0             34
#define TX0             35

#define LED_RED         13
#define LED_GREEN       14
#define LED_BLUE        15
#define LED_BUILTIN     13

#define A0              20
#define A1              21
#define A2              22
#define A3              23
#define A4              24
#define A5              25
#define A6              26
#define A7              27

#define digitalPinToPCICR(p)    (((p) >=  8 && (p) <= 12) || \
                                 ((p) >= 28 && (p) <= 30) ? (&PCICR) : ((uint8_t *)0))

// Pins PCINT7:0 are mapped to PCIE0 interrupt on bit 0 of PCICR, only PCINT8 is mapped to PCIE1.
#define digitalPinToPCICRbit(p) (((p) >=  8 && (p) <= 12) || \
                                 ((p) >= 28 && (p) <= 30) ? 0 : 1)

#define digitalPinToPCMSK(p)    (((p) >=  8 && (p) <= 12) || \
                                 ((p) >= 28 && (p) <= 30) ? (&PCMSK0) : (&PCMSK1) )

#define digitalPinToPCMSKbit(p) (((p) == 8 ) ? 0 : \
                                 (((p) >= 9 && (p) <= 12) ? ((p) - 5) : (0)))

#define digitalPinToInterrupt(p)  ( ((p) >= 0 && (p) <= 3) ? ((p) + 2) : ( ((p) == 14 || (p) == 15) ? ((p) - 14) : ( ((p) == 16 || (p) == 17) ? ((p) - 10) : NOT_AN_INTERRUPT ) ) )

#ifdef ARDUINO_MAIN

const uint16_t PROGMEM port_to_mode_PGM[] = {
  NOT_A_PORT,
  NOT_A_PORT,
  (uint16_t)&DDRB,
  NOT_A_PORT,
  (uint16_t)&DDRD,
  (uint16_t)&DDRE,
  (uint16_t)&DDRF,
  (uint16_t)&DDRG,
  NOT_A_PORT,
  NOT_A_PORT,
  NOT_A_PORT,
  NOT_A_PORT,
  NOT_A_PORT,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
  NOT_A_PORT,
  NOT_A_PORT,
  (uint16_t)&PORTB,
  NOT_A_PORT,
  (uint16_t)&PORTD,
  (uint16_t)&PORTE,
  (uint16_t)&PORTF,
  (uint16_t)&PORTG,
  NOT_A_PORT,
  NOT_A_PORT,
  NOT_A_PORT,
  NOT_A_PORT,
  NOT_A_PORT,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
  NOT_A_PIN,
  NOT_A_PIN,
  (uint16_t)&PINB,
  NOT_A_PIN,
  (uint16_t)&PIND,
  (uint16_t)&PINE,
  (uint16_t)&PINF,
  (uint16_t)&PING,
  NOT_A_PIN,
  NOT_A_PIN,
  NOT_A_PIN,
  NOT_A_PIN,
  NOT_A_PIN,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
  // PORTLIST
  // ~: PWM, *: external interrupt
  // -------------------------------------------
  PD  , // PD 0 ** D0  ** I2C_SCL*
  PD  , // PD 1 ** D1  ** I2C_SDA*
  PD  , // PD 2 ** D2  ** RX1*
  PD  , // PD 3 ** D3  ** TX1*
  PD  , // PD 4 ** D4  ** D4
  PD  , // PD 5 ** D5  ** D5
  PD  , // PD 6 ** D6  ** D6
  PD  , // PD 7 ** D7  ** D7
  PB  , // PB 0 ** D8  ** D8*
  PB  , // PB 4 ** D9  ** D9~*
  PB  , // PB 5 ** D10 ** D10~*
  PB  , // PB 6 ** D11 ** D11~*
  PB  , // PB 7 ** D12 ** D12~*
  PE  , // PE 3 ** D13 ** LED_RED~
  PE  , // PE 4 ** D14 ** LED_GREEN~*
  PE  , // PE 5 ** D15 ** LED_BLUE~*
  PE  , // PE 6 ** D16 ** D16*
  PE  , // PE 7 ** D17 ** D17*
  PE  , // PE 2 ** D18 ** D18
  PG  , // PG 5 ** D19 ** D19~
  PF  , // PF 0 ** D20 ** A0
  PF  , // PF 1 ** D21 ** A1
  PF  , // PF 2 ** D22 ** A2
  PF  , // PF 3 ** D23 ** A3
  PF  , // PF 4 ** D24 ** A4
  PF  , // PF 5 ** D25 ** A5
  PF  , // PF 6 ** D26 ** A6
  PF  , // PF 7 ** D27 ** A7
  PB  , // PB 1 ** D28 ** SCK*
  PB  , // PB 2 ** D29 ** MOSI*
  PB  , // PB 3 ** D30 ** MISO*
  PG  , // PG 0 ** D31 ** D31
  PG  , // PG 1 ** D32 ** D33
  PG  , // PG 2 ** D33 ** D33
  PE  , // PE 0 ** D34 ** RX0
  PE  , // PE 1 ** D35 ** TX0
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
  // PIN IN PORT
  // ~: PWM, *: external interrupt
  // -------------------------------------------
  _BV(PD0)  , // PD 0 ** D0  ** I2C_SCL*
  _BV(PD1)  , // PD 1 ** D1  ** I2C_SDA*
  _BV(PD2)  , // PD 2 ** D2  ** RX1*
  _BV(PD3)  , // PD 3 ** D3  ** TX1*
  _BV(PD4)  , // PD 4 ** D4  ** D4
  _BV(PD5)  , // PD 5 ** D5  ** D5
  _BV(PD6)  , // PD 6 ** D6  ** D6
  _BV(PD7)  , // PD 7 ** D7  ** D7
  _BV(PB0)  , // PB 0 ** D8  ** D8*
  _BV(PB4)  , // PB 4 ** D9  ** D9~*
  _BV(PB5)  , // PB 5 ** D10 ** D10~*
  _BV(PB6)  , // PB 6 ** D11 ** D11~*
  _BV(PB7)  , // PB 7 ** D12 ** D12~*
  _BV(PE3)  , // PE 3 ** D13 ** LED_RED~
  _BV(PE4)  , // PE 4 ** D14 ** LED_GREEN~*
  _BV(PE5)  , // PE 5 ** D15 ** LED_BLUE~*
  _BV(PE6)  , // PE 6 ** D16 ** D16*
  _BV(PE7)  , // PE 7 ** D17 ** D17*
  _BV(PE2)  , // PE 2 ** D18 ** D18
  _BV(PG5)  , // PG 5 ** D19 ** D19~
  _BV(PF0)  , // PF 0 ** D20 ** A0
  _BV(PF1)  , // PF 1 ** D21 ** A1
  _BV(PF2)  , // PF 2 ** D22 ** A2
  _BV(PF3)  , // PF 3 ** D23 ** A3
  _BV(PF4)  , // PF 4 ** D24 ** A4
  _BV(PF5)  , // PF 5 ** D25 ** A5
  _BV(PF6)  , // PF 6 ** D26 ** A6
  _BV(PF7)  , // PF 7 ** D27 ** A7
  _BV(PB1)  , // PB 1 ** D28 ** SCK*
  _BV(PB2)  , // PB 2 ** D29 ** MOSI*
  _BV(PB3)  , // PB 3 ** D30 ** MISO*
  _BV(PG0)  , // PG 0 ** D31 ** D31
  _BV(PG1)  , // PG 1 ** D32 ** D33
  _BV(PG2)  , // PG 2 ** D33 ** D33
  _BV(PE0)  , // PE 0 ** D34 ** RX0
  _BV(PE1)  , // PE 1 ** D35 ** TX0
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
  // TIMERS
  // ~: PWM, *: external interrupt
  // -------------------------------------------
  NOT_ON_TIMER  , // PD 0 ** D0  ** I2C_SCL*
  NOT_ON_TIMER  , // PD 1 ** D1  ** I2C_SDA*
  NOT_ON_TIMER  , // PD 2 ** D2  ** RX1*
  NOT_ON_TIMER  , // PD 3 ** D3  ** TX1*
  NOT_ON_TIMER  , // PD 4 ** D4  ** D4
  NOT_ON_TIMER  , // PD 5 ** D5  ** D5
  NOT_ON_TIMER  , // PD 6 ** D6  ** D6
  NOT_ON_TIMER  , // PD 7 ** D7  ** D7
  NOT_ON_TIMER  , // PB 0 ** D8  ** D8*
  TIMER2A       , // PB 4 ** D9  ** D9~*
  TIMER1A       , // PB 5 ** D10 ** D10~*
  TIMER1B       , // PB 6 ** D11 ** D11~*
  TIMER0A       , // PB 7 ** D12 ** D12~*
  TIMER3A       , // PE 3 ** D13 ** LED_RED~
  TIMER3B       , // PE 4 ** D14 ** LED_GREEN~*
  TIMER3C       , // PE 5 ** D15 ** LED_BLUE~*
  NOT_ON_TIMER  , // PE 6 ** D16 ** D16*
  NOT_ON_TIMER  , // PE 7 ** D17 ** D17*
  NOT_ON_TIMER  , // PE 2 ** D18 ** D18
  TIMER0B       , // PG 5 ** D19 ** D19~
  NOT_ON_TIMER  , // PF 0 ** D20 ** A0
  NOT_ON_TIMER  , // PF 1 ** D21 ** A1
  NOT_ON_TIMER  , // PF 2 ** D22 ** A2
  NOT_ON_TIMER  , // PF 3 ** D23 ** A3
  NOT_ON_TIMER  , // PF 4 ** D24 ** A4
  NOT_ON_TIMER  , // PF 5 ** D25 ** A5
  NOT_ON_TIMER  , // PF 6 ** D26 ** A6
  NOT_ON_TIMER  , // PF 7 ** D27 ** A7
  NOT_ON_TIMER  , // PB 1 ** D28 ** SCK*
  NOT_ON_TIMER  , // PB 2 ** D29 ** MOSI*
  NOT_ON_TIMER  , // PB 3 ** D30 ** MISO*
  NOT_ON_TIMER  , // PG 0 ** D31 ** D31
  NOT_ON_TIMER  , // PG 1 ** D32 ** D33
  NOT_ON_TIMER  , // PG 2 ** D33 ** D33
  NOT_ON_TIMER  , // PE 0 ** D34 ** RX0
  NOT_ON_TIMER  , // PE 1 ** D35 ** TX0
};

#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR         Serial
#define SERIAL_PORT_HARDWARE        Serial
#define SERIAL_PORT_HARDWARE1       Serial1
#define SERIAL_PORT_HARDWARE_OPEN   Serial1

#endif