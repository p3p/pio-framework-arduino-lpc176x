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
#include <HardwarePWM.h>
#include <SoftwarePWM.h>
#include <pwm.h>
#include <Arduino.h>
#include <time.h>

void pwm_init(void) {
  // Period defaulted to 20ms (50Hz) for compatibility with servos
  HardwarePWM::init(50);
  SoftwarePWM.init(50);
}

bool pwm_attach_pin(const pin_t pin, const uint32_t value) {
  // Hardware PWM
  if(HardwarePWM::pin_active(pin)) return true;                         // already attached to hardware channel?
  if(LPC1768_PIN_PWM(pin) && !HardwarePWM::channel_active(pin)) {       // hardware capable and channel requried by pin not in use,
    HardwarePWM::attach(pin, value);
    return true;
  }

  // Fall back on Timer3 based PWM
  if(SoftwarePWM.exists(pin)) return true; // already attached on software pin
  if(SoftwarePWM.update(pin, value)) {
    gpio_set_output(pin);
    gpio_clear(pin);
    pin_enable_feature(pin, 0);            // initialise pin for gpio output
    return true;
  }
  return false;
}

bool pwm_attached(const pin_t pin) {
  return HardwarePWM::pin_active(pin) || SoftwarePWM.exists(pin);
}

bool pwm_detach_pin(const pin_t pin) {
  // Hardware PWM capable pin and active
  if (HardwarePWM::detach(pin)) return true;
  // Fall back on Timer3 based PWM
  return SoftwarePWM.remove(pin);
}

bool pwm_write(const pin_t pin, const uint32_t value) {
  // Hardware pwm feature is active for pin
  if (HardwarePWM::pin_active(pin)) {
    HardwarePWM::set_match(pin, value);
    return true;
  }

  // Fall back on Timer3 based PWM
  if(SoftwarePWM.update(pin, value)) return true;
  return false;
}

uint32_t pwm_get_period(const pin_t pin) {
  if (HardwarePWM::pin_active(pin)) {
    return HardwarePWM::get_period();
  }
  return SoftwarePWM.get_period();
}

bool pwm_write_ratio(const pin_t pin, const uint8_t value) {
  return pwm_write(pin, map(value, 0, 255, 0, pwm_get_period(pin)));
}

bool pwm_write_ratio(const pin_t pin, const float value) {
  return pwm_write(pin, static_cast<float>(pwm_get_period(pin)) * (value > 1.0f ? 1.0 : (value < 0.0f ? 0.0f : value)));
}

bool pwm_write_us(const pin_t pin, const uint32_t value) {
  // Hardware pwm feature is active for pin
  if (HardwarePWM::pin_active(pin)) {
    HardwarePWM::set_us(pin, value);
    return true;
  }

  // Fall back on Timer3 based PWM
  if(SoftwarePWM.update(pin, value)) return true;
  return false;
}
