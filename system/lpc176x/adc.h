#pragma once

#include <LPC17xx.h>

#include <cstdint>
#include <pinmapping.h>
#include <bit_manipulation.h>

namespace LPC176x {

constexpr uint16_t analog_input_count = 8;

struct  adc_control {
  struct adc_register_block {
    volatile uint32_t control;          // 00
    volatile uint32_t global_data;      // 04
    uint32_t unused;                    // 08
    uint32_t interrupt_enable;          // 0C
    volatile uint32_t channel_data[8];  // 10,14,18,1C,20,24,28,2C
    volatile uint32_t status;           // 30
    uint32_t trim;                      // 34
  };
  static constexpr uint32_t adc_hardware_address = 0x40034000;
  [[gnu::always_inline]] inline adc_register_block& adc_reg() const {
    return *reinterpret_cast<adc_register_block*>(adc_hardware_address);
  }

  /**
   * Control Register
   */
  [[gnu::always_inline, nodiscard]] inline uint8_t selected() {
    return util::bitset_get_value(adc_reg().control, 0, 8);
  }

  [[gnu::always_inline, nodiscard]] inline bool selected(const uint8_t channel) const {
    return util::bit_test(adc_reg().control, channel);
  }

  [[gnu::always_inline]] inline void select(const uint8_t channel) {
    adc_reg().control |= (0x1 << channel);
  }

  [[gnu::always_inline]] inline void deselect(const uint8_t channel) {
    adc_reg().control &= ~(0x1 << channel);
  }

  [[gnu::always_inline]] inline void select_clear() {
    util::bitset_set_value(adc_reg().control, 0, 0, 8);
  }

  [[gnu::always_inline]] inline void select_set(const uint8_t channels) {
    util::bitset_set_value(adc_reg().control, channels, 0, 8);
  }

  [[gnu::always_inline, nodiscard]] inline uint8_t clock_divider() const {
    return util::bitset_get_value(adc_reg().control, 8, 8);
  }

  [[gnu::always_inline]] inline void clock_divider(const uint8_t divider) {
    util::bitset_set_value(adc_reg().control, divider, 8, 8);
  }

  [[gnu::always_inline]] inline void burst(const bool value) {
    util::bitset_set_value(adc_reg().control, value, 16, 1);
  }

  [[gnu::always_inline, nodiscard]] inline bool burst() const {
    return util::bitset_get_value(adc_reg().control, 16, 1);
  }

  [[gnu::always_inline]] inline void power(const bool value) {
    util::bitset_set_value(adc_reg().control, value, 21, 1);
  }

  [[gnu::always_inline, nodiscard]] inline bool power() const {
    return util::bitset_get_value(adc_reg().control, 21, 1);
  }

  [[gnu::always_inline]] inline void start(const uint8_t value) {
    util::bitset_set_value(adc_reg().control, value, 24, 3);
  }

  [[gnu::always_inline, nodiscard]] inline uint8_t start() const {
    return util::bitset_get_value(adc_reg().control, 24, 3);
  }

  [[gnu::always_inline]] inline void edge(const bool value) {
    util::bitset_set_value(adc_reg().control, value, 27, 1);
  }

  [[gnu::always_inline, nodiscard]] inline bool edge() const {
    return util::bitset_get_value(adc_reg().control, 27, 1);
  }

  /**
   * Global Data Register
   */
  [[gnu::always_inline, nodiscard]] inline uint16_t global_result() const {
    return util::bitset_get_value(adc_reg().global_data, 4, 12);
  }
  [[gnu::always_inline, nodiscard]] inline uint8_t global_channel() const {
    return util::bitset_get_value(adc_reg().global_data, 24, 3);
  }
  [[gnu::always_inline, nodiscard]] inline bool global_overrun() const {
    return util::bitset_get_value(adc_reg().global_data, 30, 1);
  }
  [[gnu::always_inline, nodiscard]] inline bool global_done() const {
    return util::bitset_get_value(adc_reg().global_data, 31, 1);
  }

  /**
   *  Interrupt Enable Register
   */
  [[gnu::always_inline, nodiscard]] inline bool interrupt_enabled(const uint8_t channel) const {
    return util::bitset_get_value(adc_reg().interrupt_enable, channel, 1);
  }
  [[gnu::always_inline]] inline void interrupt_enable(const uint8_t channel, const bool value) {
    util::bitset_set_value(adc_reg().interrupt_enable, value, channel, 1);
  }
  [[gnu::always_inline, nodiscard]] inline bool global_interrupt_enabled() const {
    return util::bitset_get_value(adc_reg().interrupt_enable, 8, 1);
  }
  [[gnu::always_inline]] inline void global_interrupt_enable(const bool value) {
    util::bitset_set_value(adc_reg().interrupt_enable, value, 8, 1);
  }

  /**
   *  Data Registers
   */
  [[gnu::always_inline, nodiscard]] inline uint16_t result(const uint8_t channel) const {
    return util::bitset_get_value(adc_reg().channel_data[channel], 4, 12);
  }
  [[gnu::always_inline, nodiscard]] inline bool overrun(const uint8_t channel) const {
    return util::bitset_get_value(adc_reg().channel_data[channel], 30, 1);
  }
  [[gnu::always_inline, nodiscard]] inline bool done(const uint8_t channel) const {
    return util::bitset_get_value(adc_reg().channel_data[channel], 31, 1);
  }

  /**
   * Status Register
   */
  [[gnu::always_inline, nodiscard]] inline uint8_t overrun() const {
    return util::bitset_get_value(adc_reg().status, 8, 8);
  }
  [[gnu::always_inline, nodiscard]] inline uint8_t done() const {
    return util::bitset_get_value(adc_reg().status, 8, 0);
  }
  [[gnu::always_inline, nodiscard]] inline bool interrupt() const {
    return util::bitset_get_value(adc_reg().status, 1, 16);
  }

  void init() {
    LPC_SC->PCONP |= (1 << 12);      // Enable CLOCK for internal ADC controller
    LPC_SC->PCLKSEL0 &= ~(0x3 << 24);
    LPC_SC->PCLKSEL0 |= (0x1 << 24); // 0: 25MHz, 1: 100MHz, 2: 50MHz, 3: 12.5MHZ to ADC clock divider
    configure();
  }
  void configure() {
    start(0);
    select_clear();
    clock_divider(255);
    burst(false);
    power(true);
  }
  /**
   * Burst mode functions
   */
  [[gnu::always_inline]] inline void burst_start() {
    start(0);
    global_interrupt_enable(0);
    burst(true);
  }
  [[gnu::always_inline]] inline void burst_stop() {
    start(0);
    select_clear();
    burst(false);
  }

  /**
   * Oneshot mode function
   */
  [[gnu::always_inline]] inline void oneshot_start(uint8_t channel) {
    select(channel);
    start(1);
  }
  [[gnu::always_inline, nodiscard]] inline bool oneshot_ready() {
    return global_done();
  }
  [[gnu::always_inline, nodiscard]] inline bool oneshot_busy() {
    return selected();
  }
  [[gnu::always_inline, nodiscard]] inline uint16_t oneshot_read() {
    select_clear();
    return global_result();
  }
};

extern adc_control& adc_hardware;

//ADC_MEDIAN_FILTER_SIZE (23)
// Higher values increase step delay (phase shift),
// (ADC_MEDIAN_FILTER_SIZE + 1) / 2 sample step delay (12 samples @ 500Hz: 24ms phase shift)
// Memory usage per ADC channel (bytes): (6 * ADC_MEDIAN_FILTER_SIZE) + 16
// 8 * ((6 * 23) + 16 ) = 1232 Bytes for 8 channels

//ADC_LOWPASS_K_VALUE  (6)
// Higher values increase rise time
// Rise time sample delays for 100% signal convergence on full range step
// (1 : 13, 2 : 32, 3 : 67, 4 : 139, 5 : 281, 6 : 565, 7 : 1135, 8 : 2273)
// K = 6, 565 samples, 500Hz sample rate, 1.13s convergence on full range step
// Memory usage per ADC channel (bytes): 4 (32 Bytes for 8 channels)

template <uint8_t ADC_LOWPASS_K_VALUE = 0, uint16_t ADC_MEDIAN_FILTER_SIZE = 0>
struct ADC {
  // Sourced from https://embeddedgurus.com/stack-overflow/tag/median-filter/
  struct MedianFilter {
    static constexpr uint32_t STOPPER = 0;        // Smaller than any datum
    struct Pair {
      Pair   *point;                          // Pointers forming list linked in sorted order
      uint16_t  value;                        // Values to sort
    };

    Pair buffer[ADC_MEDIAN_FILTER_SIZE] = {}; // Buffer of nwidth pairs
    Pair *datpoint = buffer;                  // Pointer into circular buffer of data
    Pair small = {nullptr, STOPPER};          // Chain stopper
    Pair big = {&small, 0};                   // Pointer to head (largest) of linked list.

    [[gnu::optimize("O3")]] uint16_t update(uint16_t datum) {
      Pair *successor;                        // Pointer to successor of replaced data item
      Pair *scan;                             // Pointer used to scan down the sorted list
      Pair *scanold;                          // Previous value of scan
      Pair *median;                           // Pointer to median
      uint16_t i;

      if (datum == STOPPER) {
        datum = STOPPER + 1;                  // No stoppers allowed.
      }

      if ( (++datpoint - buffer) >= ADC_MEDIAN_FILTER_SIZE) {
        datpoint = buffer;                    // Increment and wrap data in pointer.
      }

      datpoint->value = datum;                // Copy in new datum
      successor = datpoint->point;            // Save pointer to old value's successor
      median = &big;                          // Median initially to first in chain
      scanold = nullptr;                      // Scanold initially null.
      scan = &big;                            // Points to pointer to first (largest) datum in chain

      // Handle chain-out of first item in chain as special case
      if (scan->point == datpoint) {
        scan->point = successor;
      }
      scanold = scan;                         // Save this pointer and
      scan = scan->point ;                    // step down chain

      // Loop through the chain, normal loop exit via break.
      for (i = 0 ; i < ADC_MEDIAN_FILTER_SIZE; ++i) {
        // Handle odd-numbered item in chain
        if (scan->point == datpoint) {
          scan->point = successor;            // Chain out the old datum
        }

        if (scan->value < datum) {            // If datum is larger than scanned value
          datpoint->point = scanold->point;   // Chain it in here
          scanold->point = datpoint;          // Mark it chained in
          datum = STOPPER;
        }

        // Step median pointer down chain after doing odd-numbered element
        median = median->point;               // Step median pointer
        if (scan == &small) {
          break;                              // Break at end of chain
        }
        scanold = scan;                       // Save this pointer and
        scan = scan->point;                   // step down chain

        // Handle even-numbered item in chain.
        if (scan->point == datpoint) {
          scan->point = successor;
        }

        if (scan->value < datum) {
          datpoint->point = scanold->point;
          scanold->point = datpoint;
          datum = STOPPER;
        }

        if (scan == &small) {
          break;
        }

        scanold = scan;
        scan = scan->point;
      }
      return median->value;
    }
  };

  struct LowpassFilter {
    uint32_t data_delay = 0x7FF << ADC_LOWPASS_K_VALUE;
    [[gnu::always_inline]] inline uint16_t update(uint16_t value) {
      data_delay = data_delay - (data_delay >> ADC_LOWPASS_K_VALUE) + value;
      return (uint16_t)(data_delay >> ADC_LOWPASS_K_VALUE);
    }
  };

  static void enable_channel(const pin_t pin) {
    if (!pin_is_valid(pin) || !pin_has_adc(pin)) return;
    pin_enable_adc(pin);
    adc_hardware.select(pin_get_adc_channel(pin));
  }

  [[gnu::optimize("O3"), gnu::always_inline]] static inline uint16_t read(const pin_t pin) {
    if (!pin_is_valid(pin) || !pin_has_adc(pin)) return 0;
    uint8_t adc_channel = pin_get_adc_channel(pin);
    uint16_t data = adc_hardware.result(adc_channel) << 4; // 12 - 16bit conversion before filters

    if constexpr (ADC_MEDIAN_FILTER_SIZE > 0) {
      static MedianFilter median_filter[analog_input_count];
      data = median_filter[adc_channel].update(data);
    }
    if constexpr (ADC_LOWPASS_K_VALUE > 0) {
      static LowpassFilter lowpass_filter[analog_input_count];
      data = lowpass_filter[adc_channel].update(data);
    }

    return data;
  }
};
} // LPC176x
