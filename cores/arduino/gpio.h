#pragma once

#include <cstdint>

#include <registers.h>
#include <bit_manipulation.h>
#include <pinmapping.h>

inline auto gpio_port(uint8_t port) {
  constexpr std::size_t LPC_PORT_OFFSET = 0x0020;
  return util::memory_ptr<LPC_GPIO_TypeDef>(LPC_GPIO0_BASE + LPC_PORT_OFFSET * port);
}

inline void gpio_set_input(const pin_t pin) {
  util::bit_clear(gpio_port(LPC1768_PIN_PORT(pin))->FIODIR, LPC1768_PIN_PIN(pin));
}

inline void gpio_set_output(const pin_t pin) {
  util::bit_set(gpio_port(LPC1768_PIN_PORT(pin))->FIODIR, LPC1768_PIN_PIN(pin));
}

inline bool gpio_get_dir(const pin_t pin) {
  return util::bit_test(gpio_port(LPC1768_PIN_PORT(pin))->FIODIR, LPC1768_PIN_PIN(pin));
}

inline void gpio_set(const pin_t pin) {
  gpio_port(LPC1768_PIN_PORT(pin))->FIOSET = util::bit_value(LPC1768_PIN_PIN(pin));
}

inline void gpio_set_port(const uint8_t port, const uint32_t pinmap) {
  gpio_port(port)->FIOSET = pinmap;
}

inline void gpio_clear(const pin_t pin) {
  gpio_port(LPC1768_PIN_PORT(pin))->FIOCLR = util::bit_value(LPC1768_PIN_PIN(pin));
}

inline void gpio_clear_port(const uint8_t port, const uint32_t pinmap) {
  gpio_port(port)->FIOCLR = pinmap;
}

inline void gpio_set(const pin_t pin, const bool value) {
  value ? gpio_set(pin) : gpio_clear(pin);
}

inline bool gpio_get(const pin_t pin) {
  return util::bit_test(gpio_port(LPC1768_PIN_PORT(pin))->FIOPIN, LPC1768_PIN_PIN(pin));
}

inline void gpio_toggle(const pin_t pin) {
  gpio_set(pin, !gpio_get(pin));
}

constexpr uint32_t pin_feature_bits(const pin_t pin, const uint8_t feature) {
  return feature << (LPC1768_PIN_PIN(pin) < 16 ? LPC1768_PIN_PIN(pin) : LPC1768_PIN_PIN(pin) - 16) * 2;
}

constexpr auto pin_feature_reg(const pin_t pin) {
  return util::memory_ptr<uint32_t>(LPC_PINCON_BASE + (sizeof(uint32_t) * ((LPC1768_PIN_PORT(pin) * 2) + (LPC1768_PIN_PIN(pin) > 15))) );
}

constexpr void pin_enable_feature(const pin_t pin, uint8_t feature) {
  auto feature_reg = pin_feature_reg(pin);
  util::bitset_clear(*feature_reg, pin_feature_bits(pin, 0b11));
  util::bitset_set(*feature_reg, pin_feature_bits(pin, feature));
}
