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
#include <cstdint>
#include <const_functions.h>
#include <pinmapping.h>

// Get the digital pin for an analog index
pin_t analogInputToDigitalPin(const int8_t p) {
  return (util::within(p, 0, NUM_ANALOG_INPUTS) ? adc_pin_table[p] : P_NC);
}

// Return the index of a pin number
// The pin number given here is in the form ppp:nnnnn
int16_t GET_PIN_MAP_INDEX(const pin_t pin) {
  const uint16_t index = (LPC1768_PIN_PORT(pin) << 5) | LPC1768_PIN_PIN(pin);
  return (index < NUM_DIGITAL_PINS && pin_map[index] != P_NC) ? index : -1;
}

// Test whether the pin is valid
bool VALID_PIN(const pin_t p) {
  const int16_t ind = GET_PIN_MAP_INDEX(p);
  return ind >= 0 && pin_map[ind] >= 0;
}

// Get the analog index for a digital pin
int8_t DIGITAL_PIN_TO_ANALOG_PIN(const pin_t p) {
  return (VALID_PIN(p) ? LPC1768_PIN_ADC(p) : -1);
}

// Test whether the pin is PWM
bool PWM_PIN(const pin_t p) {
  return VALID_PIN(p) && LPC1768_PIN_PWM(p);
}

// Test whether the pin is interruptable
bool INTERRUPT_PIN(const pin_t p) {
  return VALID_PIN(p) && LPC1768_PIN_INTERRUPT(p);
}

// Get the pin number at the given index
pin_t GET_PIN_MAP_PIN(const int16_t ind) {
  return util::within(ind, 0, NUM_DIGITAL_PINS - 1) ? pin_map[ind] : P_NC;
}