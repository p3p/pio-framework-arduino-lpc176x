#include <LPC17xx.h>

#include <bit_manipulation.h>
#include <pinmapping.h>

pin_t tone_pin = P_NC;
volatile int32_t toggles = 0;
bool initialised_tone_timer = false;

void tone(const pin_t _pin, const uint32_t frequency, const uint32_t duration = 0) {
  if ((tone_pin != P_NC && tone_pin != _pin) || _pin == P_NC || frequency == 0) return;
  NVIC_DisableIRQ(TIMER2_IRQn);
  tone_pin = _pin;
  toggles = duration ? 2 * frequency * duration / 1000 : -1;

  if (!initialised_tone_timer) {
    LPC176x::util::bit_set(LPC_SC->PCONP, 22);                     // Power ON Timer 2
    LPC_TIM2->TCR = LPC176x::util::bit_value(1);                   // Put Timer in reset state
    LPC_TIM2->PR = ((SystemCoreClock / 4) / 1000000) - 1; // Use prescaler to set frequency
    LPC_TIM2->MCR = LPC176x::util::bitset_value(0, 1);             // Match on MR0, reset on MR0, interrupts when NVIC enables them
    NVIC_SetPriority(TIMER2_IRQn, NVIC_EncodePriority(0, 2, 0));
    initialised_tone_timer = true;
  }

  LPC_TIM2->TCR = LPC176x::util::bit_value(1);                 // Put Timer in reset state
  LPC_TIM2->MR0 = (1000000 / (2 * frequency)) - 1;    // Match value (period) to set frequency
  LPC_TIM2->TCR = LPC176x::util::bit_value(0);                 // Counter Enable

  LPC176x::pin_enable_function(_pin, LPC176x::Function::GPIO);
  LPC176x::gpio_set_output(_pin);
  LPC176x::gpio_clear(_pin);

  NVIC_EnableIRQ(TIMER2_IRQn);
}

void noTone(const pin_t _pin) {
  LPC_TIM2->TCR = LPC176x::util::bit_value(1);           // Put Timer in reset state
  NVIC_DisableIRQ(TIMER2_IRQn);
   LPC176x::gpio_clear(_pin);
  tone_pin = P_NC;
}

extern "C" [[gnu::optimize("O3")]] void TIMER2_IRQHandler(void) {
  const uint32_t interrupts = LPC_TIM2->IR;
  LPC_TIM2->IR = interrupts;  // clear all interrupts
  if (toggles != 0) {
    if(toggles > 0) toggles--;
     LPC176x::gpio_toggle(tone_pin);
  } else noTone(tone_pin);
}
