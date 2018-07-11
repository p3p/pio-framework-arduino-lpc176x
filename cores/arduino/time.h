#pragma once

#include <LPC17xx.h>
#include <const_functions.h>

namespace time {

  static inline __attribute__((__always_inline__)) void nop() {
       __asm__ __volatile__ ( "and r0,r0,r0" ::: );
  }

  template <typename T>
  static inline __attribute__((__always_inline__)) void delay_4cycles(T cy) // +1 cycle
  {
    #if ARCH_PIPELINE_RELOAD_CYCLES < 2
      #define EXTRA_NOP_CYCLES "and r0,r0,r0"
    #else
      #define EXTRA_NOP_CYCLES ""
    #endif

    __asm__ __volatile__
    (
      ".syntax unified" "\n\t" // is to prevent CM0,CM1 non-unified sintax
      "loop%=:" "\n\t"
      " subs %[cnt],#1" "\n\t"
      EXTRA_NOP_CYCLES "\n\t"
      " bne loop%=" "\n\t"
      : [cnt]"+r"(cy) // output: +r means input+output
      : // input:
      : "cc" // clobbers:
    );
  }

  template <typename T>
  static inline __attribute__((__always_inline__)) void delay_cycles(const T x) {
      if (__builtin_constant_p(x)) {
          const T max_nops = 4;
          if (x <= (max_nops)) {
            for(T i = 0; i < x; i++) nop();
          } else {
              const T rem = (x - 1) % (max_nops);
              for(T i = 0; i < rem; i++) nop();
              const T delay = (x - 1) / (max_nops);
              if (delay) delay_4cycles(x);
          }
      } else {
          delay_4cycles(x / 4);
      }
  }

  template <typename T>
  static inline __attribute__((__always_inline__)) void delay_ns(const T x) {
    delay_cycles( x * (SystemCoreClock / 1000000L) / 1000L );
  }

  template <typename T>
  static inline __attribute__((__always_inline__)) void delay_us(const T x) {
    delay_cycles( x * (SystemCoreClock / 1000000L));
  }

} // time