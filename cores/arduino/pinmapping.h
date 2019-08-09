/**
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

#pragma once

#include <cstdint>
#include <iterator>
#include <array>

#include <const_functions.h>

#include <pin_control.h>
#include <gpio.h>


#define P_NC -1
#define P0_00 0x00
#define P0_01 0x01
#define P0_02 0x02
#define P0_02_A7 P0_02
#define P0_03 0x03
#define P0_03_A6 P0_03
#define P0_04 0x04
#define P0_05 0x05
#define P0_06 0x06
#define P0_07 0x07
#define P0_08 0x08
#define P0_09 0x09
#define P0_10 0x0A
#define P0_11 0x0B
#define P0_15 0x0F
#define P0_16 0x10
#define P0_17 0x11
#define P0_18 0x12
#define P0_19 0x13
#define P0_20 0x14
#define P0_21 0x15
#define P0_22 0x16
#define P0_23 0x17
#define P0_23_A0 P0_23
#define P0_24 0x18
#define P0_24_A1 P0_24
#define P0_25 0x19
#define P0_25_A2 P0_25
#define P0_26 0x1A
#define P0_26_A3 P0_26
#define P0_27 0x1B
#define P0_28 0x1C
#define P0_29 0x1D
#define P0_30 0x1E
#define P1_00 0x20
#define P1_01 0x21
#define P1_04 0x24
#define P1_08 0x28
#define P1_09 0x29
#define P1_10 0x2A
#define P1_14 0x2E
#define P1_15 0x2F
#define P1_16 0x30
#define P1_17 0x31
#define P1_18 0x32
#define P1_19 0x33
#define P1_20 0x34
#define P1_21 0x35
#define P1_22 0x36
#define P1_23 0x37
#define P1_24 0x38
#define P1_25 0x39
#define P1_26 0x3A
#define P1_27 0x3B
#define P1_28 0x3C
#define P1_29 0x3D
#define P1_30 0x3E
#define P1_30_A4 P1_30
#define P1_31 0x3F
#define P1_31_A5 P1_31
#define P2_00 0x40
#define P2_01 0x41
#define P2_02 0x42
#define P2_03 0x43
#define P2_04 0x44
#define P2_05 0x45
#define P2_06 0x46
#define P2_07 0x47
#define P2_08 0x48
#define P2_09 0x49
#define P2_10 0x4A
#define P2_11 0x4B
#define P2_12 0x4C
#define P2_13 0x4D
#define P3_25 0x79
#define P3_26 0x7A
#define P4_28 0x9C
#define P4_29 0x9D

constexpr uint8_t NUM_DIGITAL_PINS = 160;
constexpr uint8_t NUM_ANALOG_INPUTS = 8;

// Get the digital pin for an analog index
constexpr pin_t analogInputToDigitalPin(const int8_t channel) {
  return LPC176x::pin_type::index_from_adc_channnel(channel);
}

constexpr pin_t digitalPinToInterrupt(const pin_t pin) { return pin; }