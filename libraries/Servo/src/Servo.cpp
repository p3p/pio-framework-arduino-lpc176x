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

/**
 * Based on servo.cpp - Interrupt driven Servo library for Arduino using 16 bit
 * timers- Version 2  Copyright (c) 2009 Michael Margolis.  All right reserved.
 */

/**
 * A servo is activated by creating an instance of the Servo class passing the desired pin to the attach() method.
 * The servos are pulsed in the background using the value most recently written using the write() method
 *
 * Note that analogWrite of PWM on pins associated with the timer are disabled when the first servo is attached.
 * Timers are seized as needed in groups of 12 servos - 24 servos use two timers, 48 servos will use four.
 *
 * The methods are:
 *
 * Servo - Class for manipulating servo motors connected to Arduino pins.
 *
 * attach(pin)           - Attach a servo motor to an i/o pin.
 * attach(pin, min, max) - Attach to a pin, setting min and max values in microseconds
 *                         Default min is 544, max is 2400
 *
 * write()               - Set the servo angle in degrees. (Invalid angles —over MIN_PULSE_WIDTH— are treated as µs.)
 * writeMicroseconds()   - Set the servo pulse width in microseconds.
 * move(pin, angle)      - Sequence of attach(pin), write(angle), safe_delay(servo_delay[servoIndex]).
 *                         With DEACTIVATE_SERVOS_AFTER_MOVE it detaches after servo_delay[servoIndex].
 * read()                - Get the last-written servo pulse width as an angle between 0 and 180.
 * readMicroseconds()    - Get the last-written servo pulse width in microseconds.
 * attached()            - Return true if a servo is attached.
 * detach()              - Stop an attached servo from pulsing its i/o pin.
 *
 */

#include <algorithm>
#include "Arduino.h"

#include <pwm.h>
#include <Servo.h>

ServoInfo_t Servo::servo_info[MAX_SERVOS];                  // static array of servo info structures
uint8_t Servo::ServoCount = 0;                              // the total number of attached servos

#define US_TO_PULSE_WIDTH(p) p
#define PULSE_WIDTH_TO_US(p) p
#define TRIM_DURATION 0
#define SERVO_MIN() MIN_PULSE_WIDTH  // minimum value in uS for this servo
#define SERVO_MAX() MAX_PULSE_WIDTH  // maximum value in uS for this servo

Servo::Servo() {
  if (ServoCount < MAX_SERVOS) {
    this->servoIndex = ServoCount++;                    // assign a servo index to this instance
    servo_info[this->servoIndex].pulse_width = US_TO_PULSE_WIDTH(DEFAULT_PULSE_WIDTH);   // store default values
  }
  else this->servoIndex = INVALID_SERVO;  // too many servos
}

int8_t Servo::attach(const pin_t pin) {
  return this->attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

int8_t Servo::attach(const pin_t pin, const int min, const int max) {
  if (this->servoIndex >= MAX_SERVOS || !LPC176x::pwm_attach_pin(pin, DEFAULT_PULSE_WIDTH)) return -1;
  servo_info[this->servoIndex].Pin.nbr = pin;
  servo_info[this->servoIndex].Pin.isActive = true;
  return this->servoIndex;
}

void Servo::detach() {
  servo_info[this->servoIndex].Pin.isActive = false;
  LPC176x::pwm_detach_pin(servo_info[this->servoIndex].Pin.nbr);
}

void Servo::write(int value) {
  if (value < MIN_PULSE_WIDTH) { // treat values less than 544 as angles in degrees (valid values in microseconds are handled as microseconds)
    value = map(std::clamp(value, 0, 180), 0, 180, SERVO_MIN(), SERVO_MAX());
      // odd - this sets zero degrees to 544 and 180 degrees to 2400 microseconds but the literature says
      // zero degrees should be 500 microseconds and 180 should be 2500
  }
  this->writeMicroseconds(value);
}

void Servo::writeMicroseconds(int value) {
  // calculate and store the values for the given channel
  uint8_t channel = this->servoIndex;
  if (channel < MAX_SERVOS) {  // ensure channel is valid
    // ensure pulse width is valid
    value = std::clamp(value, SERVO_MIN(), SERVO_MAX()) - (TRIM_DURATION);
    servo_info[channel].pulse_width = value;
    LPC176x::pwm_attach_pin(servo_info[this->servoIndex].Pin.nbr);
    LPC176x::pwm_write_us(servo_info[this->servoIndex].Pin.nbr, value);
  }
}

// return the value as degrees
int Servo::read() { return map(this->readMicroseconds() + 1, SERVO_MIN(), SERVO_MAX(), 0, 180); }

int Servo::readMicroseconds() {
  return (this->servoIndex == INVALID_SERVO) ? 0 : PULSE_WIDTH_TO_US(servo_info[this->servoIndex].pulse_width) + TRIM_DURATION;
}

bool Servo::attached() { return servo_info[this->servoIndex].Pin.isActive; }