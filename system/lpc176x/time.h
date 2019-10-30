#pragma once

#include <LPC17xx.h>

namespace time {

  [[gnu::always_inline, gnu::optimize("O3")]] static inline void nop() {
    __asm__ __volatile__("nop;\n\t":::);
  }

  [[gnu::always_inline, gnu::optimize("O3")]] static inline void __delay_4cycles(uint32_t cy) { // +1 cycle
    __asm__ __volatile__(
      "  .syntax unified\n\t" // is to prevent CM0,CM1 non-unified syntax
      "1:\n\t"
      "  subs %[cnt],#1\n\t" // 1
      "  nop\n\t"            // 1
      "  bne 1b\n\t"         // 1 + (1? reload)
      : [cnt]"+r"(cy)   // output: +r means input+output
      :                 // input:
      : "cc"            // clobbers:
    );
  }

  // Delay in cycles
  [[gnu::always_inline, gnu::optimize("O3")]] static inline void delay_cycles(uint32_t x) {
    if (__builtin_constant_p(x)) {
      constexpr uint32_t MAXNOPS = 4;
      if (x <= (MAXNOPS)) {
        switch (x) { case 4: nop(); case 3: nop(); case 2: nop(); case 1: nop(); }
      } else { // because of +1 cycle inside delay_4cycles
        const uint32_t rem = (x - 1) % (MAXNOPS);
        switch (rem) { case 3: nop(); case 2: nop(); case 1: nop(); }
        if ((x = (x - 1) / (MAXNOPS)))
          __delay_4cycles(x); // if need more then 4 nop loop is more optimal
      }
    } else if ((x >>= 2)) __delay_4cycles(x);
  }

  [[gnu::always_inline]] static inline void delay_ns(const uint32_t x) {
    delay_cycles( x * (F_CPU / 1000000L) / 1000L );
  }

  [[gnu::always_inline]] static inline void delay_us(const uint32_t x) {
    delay_cycles( x * (F_CPU / 1000000L));
  }

  [[gnu::always_inline]] static inline void delay_ms(const uint32_t x) {
    delay_cycles( x * (F_CPU / 1000L));
  }
} // time
