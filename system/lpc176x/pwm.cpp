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
#include <array>
#include <lpc17xx_pinsel.h>
#include <lpc17xx_clkpwr.h>
#include <time.h>
#include <const_functions.h>
#include <pinmapping.h>
#include <HardwarePWM.h>
#include <SoftwarePWM.h>
#include <pwm.h>

namespace LPC176x {
  
void pwm_init(const uint32_t frequency) {
  // Period defaulted to 20ms (50Hz) for compatibility with servos
  HardwarePWM::init(frequency);
  SoftwarePWM::init(frequency);
}

bool pwm_attached(const pin_t pin) {
  return HardwarePWM::active(pin) || SoftwarePWM::active(pin);
}

bool pwm_attach_pin(const pin_t pin, const uint32_t value, const bool force_sw) {
  if(pwm_attached(pin)) return true;    // already attached to any channel ?

  // Hardware PWM
  if(!force_sw && HardwarePWM::attach(pin, value)) {       // hardware capable and channel requried by pin not in use,
    return true;                                           // attach successfuly so return
  }

  // Fall back on Timer3 based PWM
  return SoftwarePWM::attach(pin, value);     // last chance
}

bool pwm_detach_pin(const pin_t pin) {
  // Hardware PWM capable pin and active
  if (HardwarePWM::detach(pin)) return true;
  // Fall back on Timer3 based PWM
  return SoftwarePWM::detach(pin);
}

bool pwm_write(const pin_t pin, const uint32_t value) {

  if (HardwarePWM::active(pin)) {
    HardwarePWM::set_match(pin, value);
    return true;
  }

  if (SoftwarePWM::active(pin)) {
    SoftwarePWM::set_match(pin, value);
    return true;
  }

  return false;
}

uint32_t pwm_get_period(const pin_t pin) {
  if (HardwarePWM::active(pin)) {
    return HardwarePWM::get_period();
  }

  if (SoftwarePWM::active(pin)) {
    return SoftwarePWM::get_period();
  }

  return 0;
}

bool pwm_write_ratio(const pin_t pin, const uint8_t value) {
  return pwm_write(pin, util::map(value, 0, 255, 0, pwm_get_period(pin)));
}

bool pwm_write_ratio(const pin_t pin, const float value) {
  return pwm_write(pin, static_cast<float>(pwm_get_period(pin)) * (value > 1.0f ? 1.0 : (value < 0.0f ? 0.0f : value)));
}

bool pwm_write_us(const pin_t pin, const uint32_t value) {
  if (HardwarePWM::active(pin)) {
    HardwarePWM::set_us(pin, value);
    return true;
  }

  if (SoftwarePWM::active(pin)) {
    SoftwarePWM::set_us(pin, value);
    return true;
  }

  return false;
}

bool pwm_set_frequency(const pin_t pin, const uint32_t frequency, const bool force_sw) {
  if (!pwm_attached(pin)) {
    pwm_attach_pin(pin, force_sw);
  }

  if (HardwarePWM::active(pin)) {
    HardwarePWM::set_frequency(frequency);
    return true;
  }

  if (SoftwarePWM::active(pin)) {
    SoftwarePWM::set_frequency(frequency);
    return true;
  }

  return false;
}
} // LPC176x
