#include <cstdint>

extern "C" {
  #include <LPC17xx.h>
}

#include <pinmapping.h>
#include <pwm.h>
#include <adc.h>

 __attribute__ ((weak)) void SysTick_Callback() {}

extern void setup();
extern void loop();

extern "C" {
  [[gnu::section(".noinit")]] volatile uint32_t _millis;
  [[gnu::section(".noinit")]] volatile static uint32_t _micros_per_tick_scaled; // Units: us / (2^16 ticks)

  uint32_t SysTick_Config(uint32_t ticks) {
    if (ticks > SysTick_LOAD_RELOAD_Msk) return 1;

    _micros_per_tick_scaled = (1000 << 16) / ticks;               // microseconds per system clock cycle, scaled by 2^16

    SysTick->LOAD = (ticks & SysTick_LOAD_RELOAD_Msk) - 1;        // Set reload register
    SysTick->VAL  = 0;                                            // Load the SysTick Counter Value
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;                      // Enable SysTick IRQ and SysTick Timer

    NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(0, 0, 0)); // Set Priority for Cortex-M3 System Interrupts
    return 0;
  }

  void SysTick_Handler(void) {
    ++_millis;
    SysTick_Callback();
  }

  // Runs after clock init and before global static constructors
  void SystemPostInit() {
    _millis = 0;                            // Initialise the millisecond counter value;
    SysTick_Config(SystemCoreClock / 1000); // Start millisecond global counter
  }
}

uint32_t millis() {
  return _millis;
}

uint32_t micros() {
  return (_millis * 1000) + (((SysTick->LOAD - SysTick->VAL) * _micros_per_tick_scaled) >> 16); // Shifted to remove scale factor
}

int main(void) {
  // Turn on and initialise ADC in burst mode
  LPC176x::adc_hardware.init();
  LPC176x::adc_hardware.burst_start();

  // Initialise PWM timers
  LPC176x::pwm_init();

  setup();
  for (;;) loop();
}
