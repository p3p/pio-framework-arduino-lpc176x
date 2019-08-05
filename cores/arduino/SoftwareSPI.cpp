/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016, 2017 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

/**
 * Software SPI functions originally from Arduino Sd2Card Library
 * Copyright (C) 2009 by William Greiman
 */

// --------------------------------------------------------------------------
// Software SPI
// --------------------------------------------------------------------------

/**
 * This software SPI runs at multiple rates. The SD software provides an index
 * (spiRate) of 0-6. The mapping is:
 *     0 - about 5 MHz peak (6 MHz on LPC1769)
 *     1-2 - about 2 MHz peak
 *     3 - about 1 MHz peak
 *     4 - about 500 kHz peak
 *     5 - about 250 kHz peak
 *     6 - about 125 kHz peak
 */

#include <algorithm>
#include <LPC17xx.h>
#include <gpio.h>

#include <Arduino.h>

uint8_t swSpiTransfer(uint8_t b, const uint8_t spi_speed, const pin_t sck_pin, const pin_t miso_pin, const pin_t mosi_pin) {
  for (uint8_t i = 0; i < 8; i++) {
    if (spi_speed == 0) {
      gpio_set(mosi_pin, !!(b & 0x80));
      gpio_set(sck_pin, HIGH);
      b <<= 1;
      if (miso_pin >= 0 && gpio_get(miso_pin)) b |= 1;
      gpio_set(sck_pin, LOW);
    }
    else {
      const uint8_t state = (b & 0x80) ? HIGH : LOW;
      for (uint8_t j = 0; j < spi_speed; j++)
        gpio_set(mosi_pin, state);

      for (uint8_t j = 0; j < spi_speed + (miso_pin >= 0 ? 0 : 1); j++)
        gpio_set(sck_pin, HIGH);

      b <<= 1;
      if (miso_pin >= 0 && gpio_get(miso_pin)) b |= 1;

      for (uint8_t j = 0; j < spi_speed; j++)
        gpio_set(sck_pin, LOW);
    }
  }
  return b;
}

void swSpiBegin(const pin_t sck_pin, const pin_t miso_pin, const pin_t mosi_pin) {
  pinMode(sck_pin, OUTPUT);
  if (pin_is_valid(miso_pin)) pinMode(miso_pin, INPUT);
  pinMode(mosi_pin, OUTPUT);
}

uint8_t swSpiInit(const uint8_t spiRate, const pin_t sck_pin, const pin_t mosi_pin) {
  gpio_set(mosi_pin, HIGH);
  gpio_set(sck_pin, LOW);
  return (SystemCoreClock == 120000000 ? 44 : 38) / std::pow(2, 6 - std::min(spiRate, (uint8_t)6));
}