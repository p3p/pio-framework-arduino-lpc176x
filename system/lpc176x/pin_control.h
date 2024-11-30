#pragma once

#include <array>
#include <cstdint>

namespace LPC176x {

typedef int16_t pin_t;
enum PinMode : uint8_t {
  PULLUP = 0,
  REPEATER,
  TRISTATE,
  PULLDOWN,
  NORMAL = 0,
  OPENDRAIN,
};

enum Function : uint8_t {
  GPIO,
  FUNC1,
  FUNC2,
  FUNC3,
};

struct pin_type {
  struct gpio_block {
    uint32_t reg_dir;               // 0x00
    uint32_t reg_unused[3];         // 0x04 0x08 0x0C
    uint32_t reg_mask;              // 0x10
    volatile uint32_t reg_control;  // 0x14
    volatile uint32_t reg_set;      // 0x18
    volatile uint32_t reg_clear;    // 0x1C
  };

  static constexpr uint32_t reg_gpio_base = 0x2009C000;
  static constexpr uint32_t reg_function_base = 0x4002C000;
  static constexpr uint32_t reg_mode_base = 0x4002C040;
  static constexpr uint32_t reg_mode_od_base = 0x4002C068;
  static constexpr uint32_t reg_i2c_cfg = 0x4002C07C;

  [[gnu::always_inline]] constexpr pin_type(const uint8_t port, const uint8_t pin) :
        gpio_reg_id(port),
        gpio_reg_bit(pin) {
  }
  [[gnu::always_inline]] constexpr pin_type(const pin_t pin) :
        gpio_reg_id((pin >> 0x05) & 0x07),
        gpio_reg_bit(pin & 0x1F) {
  }

  [[gnu::always_inline, nodiscard]] constexpr uint8_t port() const {
    return gpio_reg_id;
  }

  [[gnu::always_inline, nodiscard]] constexpr uint8_t bit() const {
    return gpio_reg_bit;
  }

  [[gnu::always_inline, nodiscard]] constexpr pin_t index() const {
    if(is_valid()) return (gpio_reg_id << 5) | gpio_reg_bit;
    return -1;
  }

  [[gnu::always_inline, nodiscard]] static constexpr pin_t index_from_adc_channnel(uint8_t channel) {
    return adc_pin_table[channel];
  }

  /**
   * GPIO Pin
   */
  [[gnu::always_inline]] constexpr uint32_t gpio_address() const {
    return reg_gpio_base + sizeof(gpio_block) * gpio_reg_id;
  }
  [[gnu::always_inline]] inline gpio_block& gpio_reg() const {
    return *reinterpret_cast<gpio_block*>(gpio_address());
  }
  [[gnu::always_inline]] inline void toggle() {
    gpio_reg().reg_control = gpio_reg().reg_set ^ gpio_mask();
  }
  [[gnu::always_inline]] inline void set() {
    gpio_reg().reg_set = gpio_mask();
  }
  [[gnu::always_inline]] inline void set(const bool value) {
    value ? set() : clear();
  }
  [[gnu::always_inline, nodiscard]] inline bool get() const {
    return gpio_reg().reg_control & gpio_mask();
  }
  [[gnu::always_inline]] inline void clear() {
    gpio_reg().reg_clear = gpio_mask();
  }
  [[gnu::always_inline]] inline void direction(const bool direction) {
    gpio_reg().reg_dir = direction ? gpio_reg().reg_dir | gpio_mask() : gpio_reg().reg_dir & ~gpio_mask();
  }
  [[gnu::always_inline]] inline bool direction() const {
    return gpio_reg().reg_dir & gpio_mask();
  }
  [[gnu::always_inline]] inline void input() {
    direction(0);
  }
  [[gnu::always_inline]] inline void output() {
    direction(1);
  }

  /**
   *  GPIO Port
   */
  [[gnu::always_inline]] inline void port_mask(const uint32_t mask) {
    gpio_reg().reg_mask = mask;
  }
  [[gnu::always_inline]] inline uint8_t port_mask() const {
    return gpio_reg().reg_mask;
  }
  [[gnu::always_inline]] inline void port_set(const uint32_t bitset) {
    gpio_reg().reg_set = bitset;
  }
  [[gnu::always_inline]] inline void port_clear(const uint32_t bitset) {
    gpio_reg().reg_clear = bitset;
  }
  [[gnu::always_inline]] inline uint32_t port_get() const {
    return gpio_reg().reg_control;
  }

  /**
   * Function
   */
  static constexpr std::array<uint8_t, 10> function_bits_adc{0x2, 0x1, 0x0, 0x3, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
  static constexpr std::array<uint8_t, 10> function_bits_pwm{0x0, 0x0, 0x0, 0x2, 0x1, 0x0, 0x0, 0x3, 0x0, 0x0};

  [[gnu::always_inline]] constexpr uint32_t function_reg_id() const {
    return (gpio_reg_id * 2) + (gpio_reg_bit > 15);
  }
  [[gnu::always_inline]] constexpr uint32_t function_address() const {
    return reg_function_base + (sizeof(uint32_t) * function_reg_id());
  }
  [[gnu::always_inline]] inline uint32_t& function_reg() const {
    return *reinterpret_cast<uint32_t*>(function_address());
  }
  [[gnu::always_inline]] inline void function(const uint8_t func) {
    function_reg() &= ~function_mask();
    function_reg() |= func << function_bit();
  }
  [[gnu::always_inline]] inline uint8_t function() const {
    return (function_reg() & function_mask()) >> function_bit();
  }
  [[gnu::always_inline]] inline void enable_adc() {
    mode(PinMode::TRISTATE);
    function_reg() &= ~function_mask();
    function_reg() |= function_bits_adc[function_reg_id()] << function_bit();
  }
  [[gnu::always_inline]] inline bool adc_enabled() const {
    return function() == function_bits_adc[function_reg_id()];
  }
  [[gnu::always_inline]] inline void enable_pwm() {
    function_reg() &= ~function_mask();
    function_reg() |= function_bits_pwm[function_reg_id()] << function_bit();
  }
  [[gnu::always_inline]] inline bool pwm_enabled() const {
    return function() == function_bits_pwm[function_reg_id()];
  }
  /**
   * Mode
   */
  [[gnu::always_inline]] constexpr uint32_t mode_address() const {
    return reg_mode_base + (sizeof(uint32_t) * function_reg_id());
  }
  [[gnu::always_inline]] inline uint32_t& mode_reg() const {
    return *reinterpret_cast<uint32_t*>(mode_address());
  }
  [[gnu::always_inline]] inline void mode(const PinMode pinmode) {
    mode_reg() &= ~function_mask();
    mode_reg() |= pinmode << function_bit();
  }
  [[gnu::always_inline]] inline PinMode mode() const {
    return static_cast<PinMode>((mode_reg() & function_mask()) >> function_bit());
  }

  /**
   * OpenDrain Mode
   */
  [[gnu::always_inline]] constexpr uint32_t od_mode_address() const {
    return reg_mode_od_base + sizeof(gpio_block) * gpio_reg_id;
  }
  [[gnu::always_inline]] inline uint32_t& od_mode_reg() const {
    return *reinterpret_cast<uint32_t*>(od_mode_address());
  }
  [[gnu::always_inline]] inline void mode_od(const PinMode pinmode) {
    od_mode_reg() &= ~gpio_mask();
    od_mode_reg() |= gpio_mask();
  }
  [[gnu::always_inline]] inline bool mode_od() const {
    return (od_mode_reg() & gpio_mask()) >> gpio_reg_bit;
  }

  /**
   * Function compatibility flags
   */
  [[gnu::always_inline, nodiscard]] constexpr bool is_interrupt_capable() const {
    return gpio_reg_id == 0 || gpio_reg_id == 2;
  }
  [[gnu::always_inline, nodiscard]] constexpr bool is_valid() const {
    return valid_pin();
  }
  [[gnu::always_inline, nodiscard]] constexpr bool has_adc() const {
    return adc_channel();
  }
  [[gnu::always_inline, nodiscard]] constexpr uint8_t get_adc_channel() const {
    return adc_channel() - 1;
  }
  [[gnu::always_inline, nodiscard]] constexpr uint8_t has_pwm() const {
    return pwm_channel();
  }
  [[gnu::always_inline, nodiscard]] constexpr uint8_t get_pwm_channel() const {
    return pwm_channel() - 1;
  }

private:
  const uint8_t gpio_reg_id;
  const uint8_t gpio_reg_bit;

  [[gnu::always_inline]] constexpr uint32_t gpio_mask() const {
    return 0b1 << gpio_reg_bit;
  }
  [[gnu::always_inline]] constexpr uint8_t function_id() const {
    return (gpio_reg_id * 2) + (gpio_reg_bit > 15);
  }
  [[gnu::always_inline]] constexpr uint8_t function_bit() const {
    return (gpio_reg_bit < 16 ? gpio_reg_bit : gpio_reg_bit - 16) * 2;
  }
  [[gnu::always_inline]] constexpr uint32_t function_mask() const {
    return 0b11 << function_bit();
  }

  [[gnu::always_inline]] constexpr bool valid_pin() const {
    return (gpio_reg_id == 0 && !((gpio_reg_bit >= 12 && gpio_reg_bit <= 14) ||  gpio_reg_bit == 31)) ||
            (gpio_reg_id == 1 && !(gpio_reg_bit == 02  || gpio_reg_bit == 03  || (gpio_reg_bit >= 5 && gpio_reg_bit <= 7) || (gpio_reg_bit >= 11 && gpio_reg_bit <= 13))) ||
            (gpio_reg_id == 2 &&  (gpio_reg_bit < 14)) ||
            (gpio_reg_id == 3 && (gpio_reg_bit == 25   || gpio_reg_bit == 26)) ||
            (gpio_reg_id == 4 &&  (gpio_reg_bit == 28  || gpio_reg_bit == 29));
  }
  [[gnu::always_inline]] constexpr uint8_t adc_channel() const {
    switch (gpio_reg_bit) {
      case 2:
        return gpio_reg_id == 0 ? 8 : 0;
      case 3:
        return gpio_reg_id == 0 ? 7 : 0;
      case 23:
        return gpio_reg_id == 0 ? 1 : 0;
      case 24:
        return gpio_reg_id == 0 ? 2 : 0;
      case 25:
        return gpio_reg_id == 0 ? 3 : 0;
      case 26:
        return gpio_reg_id == 0 ? 4 : 0;
      case 30:
        return gpio_reg_id == 1 ? 5 : 0;
      case 31:
        return gpio_reg_id == 1 ? 6 : 0;
    }
    return 0;
  }
  static constexpr std::array<pin_t, 8> adc_pin_table { 23, 24, 25, 26, (1 << 5) | 30, (1 << 5) | 31, 3, 2 };

  [[gnu::always_inline]] constexpr uint8_t pwm_channel() const {
    switch (gpio_reg_id) {
      case 1:
        switch (gpio_reg_bit) {
          case 18:
            return 1;
          case 20:
            return 2;
          case 21:
            return 3;
          case 23:
            return 4;
          case 24:
            return 5;
          case 26:
            return 6;
        }
        return 0;
      case 2:
        return gpio_reg_bit < 6 ? gpio_reg_bit + 1 : 0;
      case 3:
        return gpio_reg_bit == 25 ? 2 : gpio_reg_bit == 26 ? 3 : 0;
      default:
        return 0;
    }
  }
};

[[gnu::always_inline, nodiscard]] constexpr uint8_t pin_port(const pin_t pin) {
  return LPC176x::pin_type{pin}.port();
}

[[gnu::always_inline, nodiscard]] constexpr  uint8_t pin_bit(const pin_t pin) {
  return LPC176x::pin_type{pin}.bit();
}

[[gnu::always_inline, nodiscard]] constexpr pin_t pin_index(const pin_t pin) {
  return LPC176x::pin_type{pin}.index();
}

[[gnu::always_inline, nodiscard]] constexpr bool pin_is_valid(const pin_t pin) {
  return LPC176x::pin_type{pin}.is_valid();
}

[[gnu::always_inline, nodiscard]] constexpr bool pin_has_adc(const pin_t pin) {
  return LPC176x::pin_type{pin}.has_adc();
}

[[gnu::always_inline, nodiscard]] constexpr uint8_t pin_get_adc_channel(const pin_t pin) {
  return LPC176x::pin_type{pin}.get_adc_channel();
}

[[gnu::always_inline, nodiscard]] inline bool pin_adc_enabled(const pin_t pin) {
  return LPC176x::pin_type{pin}.adc_enabled();
}

[[gnu::always_inline]] inline void pin_set_mode(const pin_t pin, const PinMode mode) {
  LPC176x::pin_type{pin}.mode(mode);
}

[[gnu::always_inline, nodiscard]] inline PinMode pin_get_mode(const pin_t pin) {
  return LPC176x::pin_type{pin}.mode();
}

[[gnu::always_inline]] inline void pin_enable_adc(const pin_t pin) {
  LPC176x::pin_type{pin}.enable_adc();
}

[[gnu::always_inline]] inline void pin_enable_adc_by_channel(const uint8_t channel) {
  LPC176x::pin_type{LPC176x::pin_type::index_from_adc_channnel(channel)}.enable_adc();
}

[[gnu::always_inline, nodiscard]] inline bool pin_pwm_enabled(const pin_t pin) {
  return LPC176x::pin_type{pin}.pwm_enabled();
}

[[gnu::always_inline, nodiscard]] constexpr uint8_t pin_has_pwm(const pin_t pin) {
  return LPC176x::pin_type{pin}.has_pwm();
}

[[gnu::always_inline, nodiscard]] constexpr uint8_t pin_get_pwm_channel(const pin_t pin) {
  return LPC176x::pin_type{pin}.get_pwm_channel();
}

[[gnu::always_inline]] inline void pin_enable_pwm(const pin_t pin) {
  LPC176x::pin_type{pin}.enable_pwm();
}

[[gnu::always_inline]] inline void pin_enable_function(const pin_t pin, uint8_t function) {
  LPC176x::pin_type{pin}.function(function);
}
} // LPC176x
