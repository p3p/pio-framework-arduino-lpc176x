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

void pwm_init(void) {
  const uint32_t PR = (CLKPWR_GetPCLK(CLKPWR_PCLKSEL_PWM1) / 1000000) - 1;      // Prescalar to create 1 MHz output
  // Period defaulted to 20ms for compatibility with servos
  pwm_hardware_init(PR, 20000);
  SoftwarePWM.init(PR, 20000);
}

bool pwm_attach_pin(const pin_t pin, const uint32_t value) {
  // Hardware PWM
  if(pwm_pin_active(pin)) return true;                         // already attached to hardware channel?
  if(LPC1768_PIN_PWM(pin) && !pwm_channel_active(pin)) {       // hardware capable and channel requried by pin not in use,
    pin_enable_feature(pin, pin_feature_pwm(pin));             // attach Hardware PWM to pin
    pwm_set_match(pin, value);
    pwm_activate_channel(pin);                                 // activate the pwm channel for output on a pin
    return true;
  }

  // Fall back on Timer3 based PWM
  if(SoftwarePWM.exists(pin)) return true; // already attached on software pin
  if(SoftwarePWM.update(pin, value)) {
    pin_enable_feature(pin, 0);            // initialise pin for gpio output
    gpio_set_output(pin);
    gpio_clear(pin);
    return true;
  }
  return false;
}

bool pwm_attached(const pin_t pin) {
  return pwm_pin_active(pin) || SoftwarePWM.exists(pin);
}

bool pwm_detach_pin(const pin_t pin) {
  // Hardware PWM capable pin and active
  if (pwm_pin_active(pin)) {
    pin_enable_feature(pin, 0); // reenable gpio
    pwm_deactivate_channel(pin);
    return true;
  }

  // Fall back on Timer3 based PWM
  if(SoftwarePWM.remove(pin)) return true;

  return false;
}

bool pwm_write(const pin_t pin, const uint32_t value) {
  // Hardware pwm feature is active for pin
  if (pwm_pin_active(pin)) {
    pwm_set_match(pin, value);
    return true;
  }

  // Fall back on Timer3 based PWM
  if(SoftwarePWM.update(pin, value)) return true;
  return false;
}
