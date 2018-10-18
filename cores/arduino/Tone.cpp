
#include "pinmapping.h"

pin_t tone_pin = P_NC;
volatile int32_t toggles = 0;

void tone(const pin_t _pin, const uint32_t frequency, const uint32_t duration = 0) {
  if(tone_pin != P_NC || _pin == P_NC) return;

  tone_pin = _pin;
  toggles = duration ? 2 * frequency * duration / 1000 : -1;

  util::bit_set(LPC_SC->PCONP, 22);             // Power ON Timer 2
  LPC_TIM2->PR = ((SystemCoreClock) / 4) / (1000000) - 1; // Use prescaler to set frequency
  LPC_TIM2->MCR = util::bitset_value(0, 1);     // Match on MR0, reset on MR0, interrupts when NVIC enables them
  LPC_TIM2->MR0 = 1000000 / (2 * frequency);    // Match value (period) to set frequency
  LPC_TIM2->TCR = util::bit_value(0);           // Counter Enable

  pin_enable_feature(_pin, 0);
  gpio_set_output(_pin);
  gpio_clear(_pin);

  NVIC_SetPriority(TIMER2_IRQn, NVIC_EncodePriority(0, 2, 0));
  NVIC_EnableIRQ(TIMER2_IRQn);
}

void noTone(const pin_t _pin) {
  NVIC_DisableIRQ(TIMER2_IRQn);
  gpio_clear(_pin);
  tone_pin = P_NC;
}

extern "C" void TIMER2_IRQHandler(void) {
  static uint8_t pin_state = 0;
  const uint32_t interrupts = LPC_TIM2->IR;
  LPC_TIM2->IR = interrupts;  // clear all interrupts

  if (toggles != 0) {
    if(toggles > 0) toggles--;
    gpio_set(tone_pin, (pin_state ^= 1));
  } else noTone(tone_pin);
}
