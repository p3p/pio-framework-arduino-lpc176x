/*
 * Arduino SoftwareSerial API implementation for LPC176x by gloomyandy
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

//
// Includes
//
#include <stdint.h>
#include <stdarg.h>

#include <lpc17xx_rit.h>
#include <lpc17xx_clkpwr.h>
#include <debug_frmwrk.h>

#include <gpio.h>
#include <time.h>

#include <Arduino.h>
#include <SoftwareSerial.h>


#define FORCE_BAUD_RATE 19200
#define INTERRUPT_PRIORITY 0
#define OVERSAMPLE 3
//
// Statics
//
bool SoftwareSerial::initialised = false;
SoftwareSerial * SoftwareSerial::active_listener = NULL;
SoftwareSerial * volatile SoftwareSerial::active_out = NULL;
SoftwareSerial * volatile SoftwareSerial:: active_in = NULL;
int32_t SoftwareSerial::tx_tick_cnt = 0;
int32_t SoftwareSerial::rx_tick_cnt = 0;
uint32_t SoftwareSerial::tx_buffer = 0;
int32_t SoftwareSerial::tx_bit_cnt = 0;
uint32_t SoftwareSerial::rx_buffer = 0;
int32_t SoftwareSerial::rx_bit_cnt = -1;
uint32_t SoftwareSerial::cur_speed = 0;

//
// Private methods
//

void SoftwareSerial::setSpeed(uint32_t speed)
{
  if (speed != cur_speed) {
    NVIC_DisableIRQ(RIT_IRQn);
    if (speed != 0) {
      uint32_t clock_rate, cmp_value;
      // Get PCLK value of RIT
      clock_rate = CLKPWR_GetPCLK(CLKPWR_PCLKSEL_RIT);
      cmp_value = clock_rate/(speed*OVERSAMPLE);
      LPC_RIT->RICOMPVAL = cmp_value;
      LPC_RIT->RICOUNTER	= 0x00000000;
      /* Set timer enable clear bit to clear timer to 0 whenever
      * counter value equals the contents of RICOMPVAL
      */
      LPC_RIT->RICTRL |= (1<<1);
      NVIC_EnableIRQ(RIT_IRQn);
    }
    cur_speed = speed;
  }
}



// This function sets the current object as the "listening"
// one and returns true if it replaces another
bool SoftwareSerial::listen() {
  if (_receivePin >= 0) {
    // wait for any transmit to complete as we may change speed
    while(active_out) ;
    if (active_listener) {
      active_listener->stopListening();
    }
    rx_tick_cnt = 1;
    rx_bit_cnt = -1;
    setSpeed(_speed);
    active_listener = this;
    if (!_half_duplex)
      active_in = this;
    return true;
  }
  return false;
}

// Stop listening. Returns true if we were actually listening.
bool SoftwareSerial::stopListening() {
  if (active_listener == this) {
    // wait for any output to complete
    while (active_out) ;
    if (_half_duplex)
      setRXTX(false);
    active_listener = NULL;
    active_in = NULL;
    // turn off ints
    setSpeed(0);
    return true;
  }
  return false;
}


inline void SoftwareSerial::setTX() {
  // First write, then set output. If we do this the other way around,
  // the pin would be output low for a short while before switching to
  // output hihg. Now, it is input with pullup for a short while, which
  // is fine. With inverse logic, either order is fine.

  LPC176x::gpio_set(_transmitPin, _inverse_logic ? LOW : HIGH);
  pinMode(_transmitPin, OUTPUT);
}

inline void SoftwareSerial::setRX() {
  if (_receivePin > 0) {
    pinMode(_receivePin, _inverse_logic ? INPUT_PULLDOWN : INPUT_PULLUP); // pullup for normal logic!
  }
}

inline void SoftwareSerial::setRXTX(bool input) {
  //printf("rxtx\n");
  if (_half_duplex) {
    if (input) {
      if (active_in != this) {
        setRX();
        rx_bit_cnt = -1;
        rx_tick_cnt = 2;
        active_in = this;
      }
    }
    else {
      if (active_in == this) {
        setTX();
        active_in = NULL;
      }
    }
  }
}


[[gnu::always_inline, gnu::optimize("O3")]] inline void SoftwareSerial::send() {
  if (--tx_tick_cnt <= 0) {
    if (tx_bit_cnt++ < 10 ) {
      // send data (including start and stop bits)
      LPC176x::gpio_set(_transmitPin, tx_buffer & 1);
      tx_buffer >>= 1;
      tx_tick_cnt = OVERSAMPLE;
    }
    else {
      tx_tick_cnt = 1;
      if (_output_pending)
        active_out = NULL;
      else if (tx_bit_cnt > 10 + OVERSAMPLE*5)
      {
        if (_half_duplex && active_listener == this) {
          // setRXTX(true);
          pinMode(_receivePin, _inverse_logic ? INPUT_PULLDOWN : INPUT_PULLUP); // pullup for normal logic!
          rx_bit_cnt = -1;
          rx_tick_cnt = 2;
          active_in = this;
        }
        active_out = NULL;
      }
    }
  }
}

//
// The receive routine called by the interrupt handler
//
[[gnu::always_inline, gnu::optimize("O3")]] inline void SoftwareSerial::recv() {
  if (--rx_tick_cnt <= 0) {
    uint8_t inbit = LPC176x::gpio_get(_receivePin);
    if (rx_bit_cnt == -1) {
      // waiting for start bit
      if (!inbit) {
        // got start bit
        rx_bit_cnt = 0;
        rx_tick_cnt = OVERSAMPLE+1;
        rx_buffer = 0;
      }
      else
        rx_tick_cnt = 1;
    }
    else if (rx_bit_cnt >= 8) {
      if (inbit) {
        // stop bit read complete add to buffer
        uint8_t next = (_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF;
        if (next != _receive_buffer_head) {
          // save new data in buffer: tail points to where byte goes
          _receive_buffer[_receive_buffer_tail] = rx_buffer; // save new byte
          _receive_buffer_tail = next;
        }
        else {
          _buffer_overflow = true;
        }
      }
      rx_tick_cnt = 1;
      rx_bit_cnt = -1;
    }
    else {
      // data bits
      rx_buffer >>= 1;
      if (inbit)
        rx_buffer |= 0x80;
      rx_bit_cnt++;
      rx_tick_cnt = OVERSAMPLE;
    }
  }
}

//
// Interrupt handling
//

/* static */
[[gnu::always_inline, gnu::optimize("O3")]] inline void SoftwareSerial::handle_interrupt() {
  if (active_in) active_in->recv();
  if (active_out) active_out->send();
}

extern "C" 
[[gnu::optimize("O3")]] void RIT_IRQHandler(void) {
  LPC_RIT->RICTRL |= 1;
  SoftwareSerial::handle_interrupt();
}

//
// Constructor
//
SoftwareSerial::SoftwareSerial(pin_t receivePin, pin_t transmitPin, bool inverse_logic /* = false */) :
  _receivePin(receivePin),
  _transmitPin(transmitPin),
  _speed(0),
  _buffer_overflow(false),
  _inverse_logic(inverse_logic),
  _half_duplex(receivePin == transmitPin),
  _output_pending(0),
  _receive_buffer_tail(0),
  _receive_buffer_head(0) {
}

//
// Destructor
//
SoftwareSerial::~SoftwareSerial() {
  end();
}


//
// Public methods
//

void SoftwareSerial::begin(long speed) {
  #ifdef FORCE_BAUD_RATE
    speed = FORCE_BAUD_RATE;
  #endif
  _speed = speed;
  if (!initialised) {
    RIT_Init(LPC_RIT);
    NVIC_SetPriority(RIT_IRQn, NVIC_EncodePriority(0, INTERRUPT_PRIORITY, 0));
    initialised = true;
  }
  if (!_half_duplex) {
    setTX();
    setRX();
  }
  else
    setTX();

  listen();
  //printf("hd %d active_in %d tx %d rx %d\n", _half_duplex, active_in, _receivePin, _transmitPin);
}

void SoftwareSerial::end() {
  stopListening();
}


// Read data from buffer
int16_t SoftwareSerial::read() {
  //printf("hd %d active_in %d tx %d rx %d\n", _half_duplex, active_in, _receivePin, _transmitPin);

  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail) return -1;

  // Read from "head"
  uint8_t d = _receive_buffer[_receive_buffer_head]; // grab next byte
  _receive_buffer_head = (_receive_buffer_head + 1) % _SS_MAX_RX_BUFF;
  return d;
}

size_t SoftwareSerial::available() {
  return (_receive_buffer_tail + _SS_MAX_RX_BUFF - _receive_buffer_head) % _SS_MAX_RX_BUFF;
}

size_t SoftwareSerial::write(uint8_t b) {
  // wait for previous transmit to complete
  _output_pending = 1;
  while(active_out);
  // add start and stop bits.
  tx_buffer = b << 1 | 0x200;
  tx_bit_cnt = 0;
  tx_tick_cnt = OVERSAMPLE;
  setSpeed(_speed);
  if (_half_duplex)
    setRXTX(false);
  _output_pending = 0;
  // make us active
  active_out = this;
  return 1;
}

void SoftwareSerial::flush() {
  cli();
  _receive_buffer_head = _receive_buffer_tail = 0;
  sei();
}

int16_t SoftwareSerial::peek() {
  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;

  // Read from "head"
  return _receive_buffer[_receive_buffer_head];
}