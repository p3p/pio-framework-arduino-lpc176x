/**
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
 * The class Servo uses the PWM class to implement its functions
 *
 * The PWM1 module is only used to generate interrups at specified times. It
 * is NOT used to directly toggle pins. The ISR writes to the pin assigned to
 * that interrupt
 *
 * All PWMs use the same repetition rate - 20mS because that's the normal servo rate
 *
 */

#ifndef LPC1768_SERVO_H
#define LPC1768_SERVO_H

#include <stdint.h>
#include <pinmapping.h>

#define Servo_VERSION           2     // software api version of this library
//values in microseconds
#define MIN_PULSE_WIDTH       544     // the shortest pulse sent to a servo
#define MAX_PULSE_WIDTH      2400     // the longest pulse sent to a servo
#define DEFAULT_PULSE_WIDTH  1500     // default pulse width when servo is attached
#define REFRESH_INTERVAL    20000     // minimum time to refresh servos in microseconds

#define MAX_SERVOS              4     // Arbitrary limit, we have 6 hardware (on specific pins) and 20 software pwm channels in reality
#define INVALID_SERVO         255     // flag indicating an invalid servo index

typedef struct {
  pin_t nbr;            // a pin number from 0 to 254 (255 signals invalid pin)
  bool isActive;        // true if this channel is enabled, pin not pulsed if false
} ServoPin_t;

typedef struct {
  ServoPin_t Pin;
  unsigned int pulse_width;           // pulse width in microseconds
} ServoInfo_t;

class Servo {
  public:
    Servo();
    int8_t attach(const pin_t pin);    // attach the given pin to the next free channel, set pinMode, return channel number (-1 on fail)
    int8_t attach(const pin_t pin, const int min, const int max); // as above but also sets min and max values for writes.
    void detach();
    void write(int value);             // if value is < 200 it is treated as an angle, otherwise as pulse width in microseconds
    void writeMicroseconds(int value); // write pulse width in microseconds
    int read();                        // returns current pulse width as an angle between 0 and 180 degrees
    int readMicroseconds();            // returns current pulse width in microseconds for this servo (was read_us() in first release)
    bool attached();                   // return true if this servo is attached, otherwise false

  protected:
    uint8_t servoIndex;               // index into the channel data for this servo
    static ServoInfo_t servo_info[MAX_SERVOS]; // static array of servo info structures
    static uint8_t ServoCount;             // the total number of attached servos
};

#endif // LPC1768_SERVO_H
