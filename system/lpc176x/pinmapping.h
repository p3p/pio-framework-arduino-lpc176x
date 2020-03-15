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

#include <pin_control.h>
#include <gpio.h>

using LPC176x::pin_t;

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
#define P0_12 0x0B
#define P0_13 0x0C
#define P0_14 0x0D
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
#define P1_02 0x22
#define P1_03 0x23
#define P1_04 0x24
#define P1_05 0x25
#define P1_06 0x26
#define P1_07 0x27
#define P1_08 0x28
#define P1_09 0x29
#define P1_10 0x2A
#define P1_11 0x2B
#define P1_12 0x2C
#define P1_13 0x2D
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
#define P2_14 0x4E
#define P2_15 0x4F
#define P2_16 0x50
#define P2_17 0x51
#define P2_18 0x52
#define P2_19 0x53
#define P2_20 0x54
#define P2_21 0x55
#define P2_22 0x56
#define P2_23 0x57
#define P2_24 0x58
#define P2_25 0x59
#define P2_26 0x5A
#define P2_27 0x5B
#define P2_28 0x5C
#define P2_29 0x5D
#define P2_30 0x5E
#define P2_31 0x5F

#define P3_00 0x60
#define P3_01 0x61
#define P3_02 0x62
#define P3_03 0x63
#define P3_04 0x64
#define P3_05 0x65
#define P3_06 0x66
#define P3_07 0x67
#define P3_08 0x68
#define P3_09 0x69
#define P3_10 0x6A
#define P3_11 0x6B
#define P3_12 0x6C
#define P3_13 0x6D
#define P3_14 0x6E
#define P3_15 0x6F
#define P3_16 0x70
#define P3_17 0x71
#define P3_18 0x72
#define P3_19 0x73
#define P3_20 0x74
#define P3_21 0x75
#define P3_22 0x76
#define P3_23 0x77
#define P3_24 0x78
#define P3_25 0x79
#define P3_26 0x7A
#define P3_27 0x7B
#define P3_28 0x7C
#define P3_29 0x7D
#define P3_30 0x7E
#define P3_31 0x7F

#define P4_00 0x80
#define P4_01 0x81
#define P4_02 0x82
#define P4_03 0x83
#define P4_04 0x84
#define P4_05 0x85
#define P4_06 0x86
#define P4_07 0x87
#define P4_08 0x88
#define P4_09 0x89
#define P4_10 0x8A
#define P4_11 0x8B
#define P4_12 0x8C
#define P4_13 0x8D
#define P4_14 0x8E
#define P4_15 0x8F
#define P4_16 0x90
#define P4_17 0x91
#define P4_18 0x92
#define P4_19 0x93
#define P4_20 0x94
#define P4_21 0x95
#define P4_22 0x96
#define P4_23 0x97
#define P4_24 0x98
#define P4_25 0x99
#define P4_26 0x9A
#define P4_27 0x9B
#define P4_28 0x9C
#define P4_29 0x9D
#define P4_30 0x9E
#define P4_31 0x9F

#define P5_00 0xA0
#define P5_01 0xA1
#define P5_02 0xA2
#define P5_03 0xA3
#define P5_04 0xA4
