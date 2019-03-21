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

#include <lpc17xx_clkpwr.h>
#include <lpc17xx_pwm.h>
#include <gpio.h>

class HardwarePWM {
  // return the bits to attach the PWM hardware depending on port using a lookup table
  [[nodiscard]] static constexpr int8_t pwm_feature_index(const pin_t pin) noexcept {
    constexpr std::array<int8_t, 5> lookup {-1, 2, 1, 3, -1};
    return lookup[LPC1768_PIN_PORT(pin)];
  }

  // return a reference to a PWM timer register using a lookup table as they are not contiguous
  [[nodiscard]] static constexpr uint32_t match_register_lookup(const pin_t pin) noexcept {
    constexpr uint32_t MR0_OFFSET = 6, MR4_OFFSET = 16;
    return LPC_PWM1_BASE + (LPC1768_PIN_PWM(pin) > 3 ? (MR4_OFFSET + LPC1768_PIN_PWM(pin) - 4) : (MR0_OFFSET + LPC1768_PIN_PWM(pin))) * sizeof(uint32_t);
  }

  // return a reference to a PWM timer register using a lookup table as they are not contiguous
  [[nodiscard]] static constexpr auto match_register_ptr(const pin_t pin) noexcept {
    return util::memory_ptr<uint32_t>(match_register_lookup(pin));
  }

  // generate a unique bit for each hardware PWM capable pin
  [[nodiscard]] static constexpr uint8_t get_pin_id(const pin_t pin) noexcept {
    return (LPC1768_PIN_PORT(pin) * 6) + (LPC1768_PIN_PWM(pin) - 1);
  }

  // update the bitset an activate hardware pwm channel for output
  static inline void activate_channel(const pin_t pin) {
    util::bit_set(active_pins, get_pin_id(pin));         // mark the pin as active
    util::bit_clear(idle_pins, get_pin_id(pin));
    util::bit_set(LPC_PWM1->PCR, 8 + LPC1768_PIN_PWM(pin));  // turn on the pins PWM output (8 offset + PWM channel)
    pin_enable_feature(pin, pwm_feature_index(pin));
  }

  // update the bitset and deactivate the hardware pwm channel
  static inline void deactivate_channel(const pin_t pin) {
    util::bit_clear(active_pins, get_pin_id(pin));      // mark pin as inactive
    if(!channel_active(pin)) util::bit_clear(LPC_PWM1->PCR, 8 + LPC1768_PIN_PWM(pin)); // turn off the PWM output
  }

  // update the bitset and deactivate the hardware pwm channel
  static inline void set_idle(const pin_t pin) {
    gpio_set_output(pin); // used when at 0 duty cycle
    util::bit_set(idle_pins, get_pin_id(pin));      // mark pin as inactive
    pin_enable_feature(pin, 0);
    gpio_clear(pin);
  }

    // return true if a pwm channel is already attached to a pin
  [[nodiscard]] static constexpr bool channel_active(const pin_t pin) noexcept {
    const uint32_t channel = LPC1768_PIN_PWM(pin) - 1;
    return LPC1768_PIN_PWM(pin) && util::bitset_mask(active_pins, util::bitset_value(6 + channel, 2 * 6 + channel, 3 * 6 + channel));
  }

public:
  //static void init(const uint32_t prescale, const uint32_t period) {
  static void init(const uint32_t frequency) {
    // Power on the peripheral
    CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCPWM1, ENABLE);
    CLKPWR_SetPCLKDiv (CLKPWR_PCLKSEL_PWM1, CLKPWR_PCLKSEL_CCLK_DIV_4);

    // Make sure it is in a clean state
    LPC_PWM1->IR = 0xFF & PWM_IR_BITMASK;
    LPC_PWM1->TCR = 0;
    LPC_PWM1->CTCR = 0;
    LPC_PWM1->MCR = 0;
    LPC_PWM1->CCR = 0;
    LPC_PWM1->PCR &= 0xFF00;
    LPC_PWM1->LER = 0;

    // No clock prescaler
    LPC_PWM1->PR = 0;

    // Configured to reset TC if it matches MR0, No interrupts
    LPC_PWM1->MCR = util::bit_value(1);

    // Set the period using channel 0 before enabling peripheral
    LPC_PWM1->MR0 = (CLKPWR_GetPCLK(CLKPWR_PCLKSEL_PWM1) / frequency) - 1;
    LPC_PWM1->LER = util::bit_value(0); // if only latching worked

    // Enable PWM mode
    // TODO: this is very unreliable appears to randomly miss latches thus not changing the duty cycle
    // disabling PWM latch mode at least gives reliable (bit 3)
    //LPC_PWM1->TCR = util::bitset_value(0, 3);      //  Turn on PWM latch mode and Enable counters
    LPC_PWM1->TCR = util::bitset_value(0);
  }

  [[nodiscard]] static constexpr bool available(const pin_t pin) noexcept {
    return LPC1768_PIN_PWM(pin) && !channel_active(pin);
  }

  // return true if a pin is already attached to PWM hardware
  [[nodiscard]] static constexpr bool active(const pin_t pin) noexcept {
    return LPC1768_PIN_PWM(pin) && util::bit_test(active_pins, get_pin_id(pin));
  }

  static inline void set_frequency(const uint32_t frequency) {
    set_period(CLKPWR_GetPCLK(CLKPWR_PCLKSEL_PWM1) / frequency);
  }

  static inline void set_period(const uint32_t period) {
    LPC_PWM1->TCR = util::bit_value(1);
    LPC_PWM1->MR0 = period - 1;  // TC resets every period cycles
    LPC_PWM1->LER = util::bit_value(0);
    LPC_PWM1->TCR = util::bitset_value(0);
  }

  static inline uint32_t get_period() {
    return LPC_PWM1->MR0 + 1;
  }

  static inline void set_us(const pin_t pin, const uint32_t value) {
    set_match(pin, (CLKPWR_GetPCLK(CLKPWR_PCLKSEL_PWM1) / 1000000) * value);
  }

  // update the match register for a channel and set the latch to update on next period
  static inline void set_match(const pin_t pin, const uint32_t value) {
    //work around for bug if MR1 == MR0
    *match_register_ptr(pin) = value == LPC_PWM1->MR0 ? value + 1 : value;
    // tried to work around latch issue by always setting all bits, was unsuccessful
    LPC_PWM1->LER = util::bit_value(LPC1768_PIN_PWM(pin));

    // At 0 duty cycle hardware pwm outputs 1 cycle pulses
    // Work around it by disabling the pwm output and setting the pin low util the duty cycle is updated
    if(value == 0) {
      set_idle(pin);
    } else if(util::bit_test(idle_pins, get_pin_id(pin))) {
      pin_enable_feature(pin, pwm_feature_index(pin));
      util::bit_clear(idle_pins, get_pin_id(pin));
    }
  }

  static inline bool attach(const pin_t pin, const uint32_t value) {
    if(!available(pin)) return false;
    set_match(pin, value);
    activate_channel(pin);
    return true;
  }

  static inline bool detach(const pin_t pin) {
    if (active(pin)) {
      pin_enable_feature(pin, 0); // reenable gpio
      gpio_clear(pin);
      deactivate_channel(pin);
      return true;
    }
    return false;
  }

private:
  // 32bit bitset used to track whether a pin is activly using hardware pwm
  static uint32_t active_pins;
  static uint32_t idle_pins;
};

#endif // _HARDWARE_PWM_H_
