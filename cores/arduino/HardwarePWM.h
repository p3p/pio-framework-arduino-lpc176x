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

#ifndef _HARDWARE_PWM_H_
#define _HARDWARE_PWM_H_

#include <time.h>
#include <lpc17xx_pwm.h>
#include <pinmapping.h>

// 32bit bitset used to track whether a pin is activly using hardware pwm
extern uint32_t active_pwm_pins;
extern uint32_t idle_pwm_pins;

void pwm_hardware_init(const uint32_t prescale, const uint32_t period);

// return the bits to attach the PWM hardware depending on port using a lookup table
[[nodiscard]] constexpr int8_t pin_feature_pwm(const pin_t pin) noexcept {
  constexpr std::array<int8_t, 5> lookup {-1, 2, 1, 3, -1};
  return lookup[LPC1768_PIN_PORT(pin)];
}

// return a reference to a PWM timer register using a lookup table as they are not contiguous
[[nodiscard]] constexpr uint32_t pwm_match_lookup(const pin_t pin) noexcept {
  constexpr uint32_t MR0_OFFSET = 6, MR4_OFFSET = 16;
  return LPC_PWM1_BASE + (LPC1768_PIN_PWM(pin) > 3 ? (MR4_OFFSET + LPC1768_PIN_PWM(pin) - 4) : (MR0_OFFSET + LPC1768_PIN_PWM(pin))) * sizeof(uint32_t);
}

// return a reference to a PWM timer register using a lookup table as they are not contiguous
[[nodiscard]] constexpr auto pin_pwm_match(const pin_t pin) noexcept {
   return util::memory_ptr<uint32_t>(pwm_match_lookup(pin));
}

// generate a unique bit for each hardware PWM capable pin
[[nodiscard]] constexpr uint8_t pwm_pin_id(const pin_t pin) noexcept {
  return (LPC1768_PIN_PORT(pin) * 6) + (LPC1768_PIN_PWM(pin) - 1);
}

// return true if a pwm channel is already attached to a pin
[[nodiscard]] constexpr bool pwm_channel_active(const pin_t pin) noexcept {
  const uint32_t channel = LPC1768_PIN_PWM(pin) - 1;
  return LPC1768_PIN_PWM(pin) && util::bitset_mask(active_pwm_pins, util::bitset_value(6 + channel, 2 * 6 + channel, 3 * 6 + channel));
}

// return true if a pin is already attached to PWM hardware
[[nodiscard]] constexpr bool pwm_pin_active(const pin_t pin) noexcept {
  return LPC1768_PIN_PWM(pin) && util::bit_test(active_pwm_pins, pwm_pin_id(pin));
}

[[gnu::always_inline]] inline void pwm_set_period(const uint32_t period) {
  LPC_PWM1->TCR = util::bit_value(1);
  LPC_PWM1->MR0 = period - 1;  // TC resets every period cycles
  LPC_PWM1->LER = util::bit_value(0);
  LPC_PWM1->TCR = util::bitset_value(0);
}

// update the bitset an activate hardware pwm channel for output
[[gnu::always_inline]] inline void pwm_activate_channel(const pin_t pin) {
  util::bit_set(active_pwm_pins, pwm_pin_id(pin));         // mark the pin as active
  util::bit_clear(idle_pwm_pins, pwm_pin_id(pin));
  util::bit_set(LPC_PWM1->PCR, 8 + LPC1768_PIN_PWM(pin));  // turn on the pins PWM output (8 offset + PWM channel)
  pin_enable_feature(pin, pin_feature_pwm(pin));
}

// update the bitset and deactivate the hardware pwm channel
[[gnu::always_inline]] inline void pwm_deactivate_channel(const pin_t pin) {
  util::bit_clear(active_pwm_pins, pwm_pin_id(pin));      // mark pin as inactive
  if(!pwm_channel_active(pin)) util::bit_clear(LPC_PWM1->PCR, 8 + LPC1768_PIN_PWM(pin)); // turn off the PWM output
}

// update the bitset and deactivate the hardware pwm channel
[[gnu::always_inline]] inline void pwm_idle_channel(const pin_t pin) {
  gpio_set_output(pin); // used when at 0 duty cycle
  util::bit_set(idle_pwm_pins, pwm_pin_id(pin));      // mark pin as inactive
  pin_enable_feature(pin, 0);
  gpio_clear(pin);
}

// update the match register for a channel and set the latch to update on next period
[[gnu::always_inline]] inline void pwm_set_match(const pin_t pin, const uint32_t value) {
  //work around for bug if MR1 == MR0
  *pin_pwm_match(pin) = value == LPC_PWM1->MR0 ? value + 1 : value;
  // tried to work around latch issue by always setting all bits, was unsuccessful
  LPC_PWM1->LER = util::bit_value(LPC1768_PIN_PWM(pin));

  // At 0 duty cycle hardware pwm outputs 1 cycle pulses
  // Work around it by disabling the pwm output and setting the pin low util the duty cycle is updated
  if(value == 0) {
    pwm_idle_channel(pin);
  } else if(util::bit_test(idle_pwm_pins, pwm_pin_id(pin))) {
    pin_enable_feature(pin, pin_feature_pwm(pin));
    util::bit_clear(idle_pwm_pins, pwm_pin_id(pin));
  }
}

[[gnu::always_inline]] inline void pwm_hardware_attach(pin_t pin, uint32_t value) {
  pwm_set_match(pin, value);
  pwm_activate_channel(pin);
}

[[gnu::always_inline]] inline bool pwm_hardware_detach(const pin_t pin) {
  if (pwm_pin_active(pin)) {
    pin_enable_feature(pin, 0); // reenable gpio
    gpio_clear(pin);
    pwm_deactivate_channel(pin);
    return true;
  }
  return false;
}

#endif // _HARDWARE_PWM_H_
