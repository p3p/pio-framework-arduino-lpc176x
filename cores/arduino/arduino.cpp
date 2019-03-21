/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include <cstdio>

#include <lpc17xx_pinsel.h>

#include <time.h>
#include <const_functions.h>
#include <adc.h>
#include <pwm.h>
#include <gpio.h>

#include <Arduino.h>

extern uint64_t _millis;

// Interrupts
void cli(void) { __disable_irq(); } // Disable
void sei(void) { __enable_irq(); }  // Enable

void noInterrupts() { __disable_irq(); } // Disable
void interrupts() { __enable_irq(); }  // Enable

// Time functions
void _delay_ms(const int delay_ms) {
  delay(delay_ms);
}

uint32_t millis() {
  return _millis;
}

uint32_t micros() {
  return (_millis * 1000) + ((SysTick->LOAD - SysTick->VAL) / (SystemCoreClock / 1000000) );
}

// This is required for some Arduino libraries we are using
void delayMicroseconds(uint32_t us) {
  time::delay_us(us);
}

void delay(const int msec) {
  time::delay_ms(msec);
}

// IO functions
// As defined by Arduino INPUT(0x0), OUTPUT(0x1), INPUT_PULLUP(0x2)
void pinMode(const pin_t pin, const uint8_t mode) {
  if (!VALID_PIN(pin)) return;

  PINSEL_CFG_Type config = { LPC1768_PIN_PORT(pin),
                             LPC1768_PIN_PIN(pin),
                             PINSEL_FUNC_0,
                             PINSEL_PINMODE_TRISTATE,
                             PINSEL_PINMODE_NORMAL };
  switch (mode) {
    case INPUT:
      gpio_set_input(pin);
      break;
    case OUTPUT:
      gpio_set_output(pin);
      break;
    case INPUT_PULLUP:
      gpio_set_input(pin);
      config.Pinmode = PINSEL_PINMODE_PULLUP;
      break;
    case INPUT_PULLDOWN:
      gpio_set_input(pin);
      config.Pinmode = PINSEL_PINMODE_PULLDOWN;
      break;
    default: return;
  }
  PINSEL_ConfigPin(&config);
}

void digitalWrite(pin_t pin, uint8_t pin_status) {
  if (!VALID_PIN(pin)) return;

  gpio_set(pin, pin_status);
  pinMode(pin, OUTPUT);  // Set pin mode on every write (Arduino version does this)

  /**
   * Must be done AFTER the output state is set. Doing this before will cause a
   * 2uS glitch if writing a "1".
   *
   * When the Port Direction bit is written to a "1" the output is immediately set
   * to the value of the FIOPIN bit which is "0" because of power up defaults.
   */
}

bool digitalRead(pin_t pin) {
  if (!VALID_PIN(pin)) return false;
  return gpio_get(pin);
}

void analogWrite(pin_t pin, int pwm_value) {  // 1 - 254: pwm_value, 0: LOW, 255: HIGH
  if (!VALID_PIN(pin)) return;

  util::limit(pwm_value, 0, 255);
  if (pwm_attach_pin(pin)) {
    pwm_write_ratio(pin, (uint8_t)pwm_value);  // map 1-254 onto PWM range
  } else {
    digitalWrite(pin, pwm_value);  // treat as a digital pin if out of channels
  }
}

uint16_t analogRead(pin_t adc_pin) {
  LPC176x::ADC<>::start_conversion(adc_pin);
  while (!LPC176x::ADC<>::finished_conversion());  // Wait for conversion to finish
  return LPC176x::ADC<>::get_result();
}

// **************************
// Persistent Config Storage
// **************************

void eeprom_write_byte(uint8_t *pos, unsigned char value) { }

uint8_t eeprom_read_byte(uint8_t * pos) { return '\0'; }

void eeprom_read_block(void *__dst, const void *__src, size_t __n) { }

void eeprom_update_block(const void *__src, void *__dst, size_t __n) { }

char *dtostrf (double __val, signed char __width, unsigned char __prec, char *__s) {
  char format_string[20];
  snprintf(format_string, 20, "%%%d.%df", __width, __prec);
  sprintf(__s, format_string, __val);
  return __s;
}

int32_t random(int32_t max) {
  return rand() % max;
}

int32_t random(int32_t min, int32_t max) {
  return min + rand() % (max - min);
}

void randomSeed(uint32_t value) {
  srand(value);
}
