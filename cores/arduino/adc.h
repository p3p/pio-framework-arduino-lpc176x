#pragma once

#include <stdint.h>
#include <LPC17xx.h>
#include <pinmapping.h>

namespace LPC176x {

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
  static constexpr uint32_t ADC_DONE = 0x80000000;
  static constexpr uint32_t ADC_OVERRUN = 0x40000000;

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

    uint16_t update(uint16_t datum) {
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
    uint32_t data_delay = 0;
    uint16_t update(uint16_t value) {
      data_delay = data_delay - (data_delay >> ADC_LOWPASS_K_VALUE) + value;
      return (uint16_t)(data_delay >> ADC_LOWPASS_K_VALUE);
    }
  };

  static void init() {
    LPC_SC->PCONP |= (1 << 12);      // Enable CLOCK for internal ADC controller
    LPC_SC->PCLKSEL0 &= ~(0x3 << 24);
    LPC_SC->PCLKSEL0 |= (0x1 << 24); // 0: 25MHz, 1: 100MHz, 2: 50MHz, 3: 12.5MHZ to ADC clock divider
    LPC_ADC->ADCR = (0 << 0)         // SEL: 0 = no channels selected
                  | (0xFF << 8)      // select slowest clock for A/D conversion 150 - 190 uS for a complete conversion
                  | (0 << 16)        // BURST: 0 = software control
                  | (0 << 17)        // CLKS: not applicable
                  | (1 << 21)        // PDN: 1 = operational
                  | (0 << 24)        // START: 0 = no start
                  | (0 << 27);       // EDGE: not applicable
  }

  static void enable_channel(const int ch) {
    const pin_t pin = analogInputToDigitalPin(ch);

    if (pin == -1) return;

    const int8_t pin_port = LPC1768_PIN_PORT(pin),
           pin_port_pin = LPC1768_PIN_PIN(pin),
           pinsel_start_bit = pin_port_pin > 15 ? 2 * (pin_port_pin - 16) : 2 * pin_port_pin;

    const uint8_t pin_sel_register = (pin_port == 0 && pin_port_pin <= 15) ? 0 :
                                      pin_port == 0                        ? 1 :
                                      pin_port == 1                        ? 3 : 10;

    switch (pin_sel_register) {
      case 1:
        LPC_PINCON->PINSEL1 &= ~(0x3 << pinsel_start_bit);
        LPC_PINCON->PINSEL1 |=  (0x1 << pinsel_start_bit);
        break;
      case 3:
        LPC_PINCON->PINSEL3 &= ~(0x3 << pinsel_start_bit);
        LPC_PINCON->PINSEL3 |=  (0x3 << pinsel_start_bit);
        break;
      case 0:
        LPC_PINCON->PINSEL0 &= ~(0x3 << pinsel_start_bit);
        LPC_PINCON->PINSEL0 |=  (0x2 << pinsel_start_bit);
        break;
    };
  }

  static inline void start_conversion(const uint8_t ch) {
    if (analogInputToDigitalPin(ch) == -1) return;
    LPC_ADC->ADCR &= ~0xFF; // Reset
    LPC_ADC->ADCR |= (1 << ch); // Select Channel
    LPC_ADC->ADCR |= (1 << 24); // Start conversion
  }

  static inline bool finished_conversion(void) {
    return LPC_ADC->ADGDR & ADC_DONE;
  }

  static uint16_t get_result(void) {
    const uint32_t adgdr = LPC_ADC->ADGDR;
    LPC_ADC->ADCR &= ~(1 << (24)); // Stop conversion
    if (adgdr & ADC_OVERRUN) return 0;

    uint16_t data = (adgdr >> 4) & 0xFFF;      // copy the 12bit data value

    if constexpr (ADC_MEDIAN_FILTER_SIZE > 0 || ADC_LOWPASS_K_VALUE > 0) {
      uint8_t adc_channel = (adgdr >> 24) & 0x7; // copy the  3bit used channel

      if constexpr (ADC_MEDIAN_FILTER_SIZE > 0) {
        static MedianFilter median_filter[NUM_ANALOG_INPUTS];
        data = median_filter[adc_channel].update(data);
      }

      if constexpr (ADC_LOWPASS_K_VALUE > 0) {
        static LowpassFilter lowpass_filter[NUM_ANALOG_INPUTS];
        data = lowpass_filter[adc_channel].update(data);
      }
    }

    return ((data >> 2) & 0x3FF);    // return 10bit value as Marlin expects
  }
};

}