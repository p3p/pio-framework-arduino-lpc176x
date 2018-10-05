#include <SoftwarePWM.h>

SoftwarePwmTable<PWM_MAX_SOFTWARE_CHANNELS> SoftwarePWM {};

struct PwmFrameItem { pin_t pin = P_NC; uint32_t match = 0; };
std::array<PwmFrameItem, PWM_MAX_SOFTWARE_CHANNELS> pwm_frame;  // need to cache data for the frame to avoid out of frame transitions (160 bytes @ 20 software pwm pins!)

struct PinRange { std::array<PwmFrameItem, PWM_MAX_SOFTWARE_CHANNELS>::iterator start, end; };
std::array<PinRange, 3> MR_pin = {};

volatile uint32_t* MR = &LPC_TIM3->MR1; // base pointer to the match register MR1 - MR3, indexed MR[0-2]
auto next_pin = pwm_frame.begin();
auto end_pin = pwm_frame.end();

/*
  (times do not include overhead for the ISR triggering but adds gpio overhead)
  1 Software PWM channel:
    3us period ISR
    1.3 match ISR
  For 20 software PWM signals
    Max ~9 us period ISR to build the frame from the linked list
    1.3 us match ISR, if all 20 software PWM channels fire within the same interrupt it takes ~5us
*/
extern "C" void TIMER3_IRQHandler(void) {
  constexpr std::array<uint8_t, 3> MR_int {3, 6, 9};
  const uint32_t interrupts = LPC_TIM3->IR;
  LPC_TIM3->IR = interrupts;  // clear all interrupts

  if (util::bit_test(interrupts, 0)) {  // MR0 interrupt
    // reset all channels
    auto frame_it = pwm_frame.begin();
    // iterate through the attached pin list building (latching in) this frames data list, execution increases linearly with number of active software pins
    for (SwPwmData* it = SoftwarePWM.swpwm_table_head; it != nullptr; it = it->next) {
      if(it->value > 0) {
        gpio_set(it->pin);  // set the pin high for the start of this frame
        frame_it->pin = it->pin;
        frame_it->match = it->value;
        frame_it = std::next(frame_it);
      } else {
        gpio_clear(it->pin); // make sure the pin is always low @ 0 pulsewidth
      }
    }
    end_pin = frame_it;

    next_pin = pwm_frame.begin();
    // set the initial match conditions for the 3 channels
    for(uint8_t MR_reg = 0; MR_reg < 3; MR_reg++) {
      if(next_pin != end_pin) {
        MR_pin[MR_reg].start = next_pin;                  // the first pin to toggle on this channels interrupt
        // Write new value to match register with safety offset, Timers interrupt only trigger if MR == TC
        MR[MR_reg] = next_pin->match > LPC_TIM3->TC + PWM_MATCH_OFFSET ? next_pin->match : LPC_TIM3->TC + PWM_MATCH_OFFSET;
        util::bit_set(LPC_TIM3->MCR, MR_int[MR_reg]);     // enable this channels interrupt
        const uint32_t max_threshhold = MR[MR_reg];       // copy to a "faster" non volatile variable
        // skip until match value outside threshhold for next interrupt triggered by this match register
        while(next_pin != end_pin && next_pin->match <= max_threshhold) next_pin = std::next(next_pin);
        MR_pin[MR_reg].end = next_pin;                    // marks the end of the block of pins to toggle
      } else break;
    }
  }

  // set the gpio and fill the next match for each channel if available
  for(uint8_t MR_reg = 0; MR_reg < 3; MR_reg++) {
    if (util::bit_test(interrupts, MR_reg + 1)) {
      // clear all pins required for this match interrupt
      while(MR_pin[MR_reg].start != MR_pin[MR_reg].end) {
        gpio_clear(MR_pin[MR_reg].start->pin);
        MR_pin[MR_reg].start = std::next(MR_pin[MR_reg].start);
      }
      // calculate next pin(s) to match for
      if(next_pin != end_pin) {
        MR_pin[MR_reg].start = next_pin;
        MR[MR_reg] = next_pin->match > LPC_TIM3->TC + PWM_MATCH_OFFSET ? next_pin->match : LPC_TIM3->TC + PWM_MATCH_OFFSET;
        const uint32_t max_threshhold = MR[MR_reg];
        // skip until match value outside threshhold for next interrupt triggered by this match register
        while(next_pin != end_pin && next_pin->match <= max_threshhold) next_pin = std::next(next_pin);
        MR_pin[MR_reg].end = next_pin;
      } else {
        // no more pins this frame, disable this channels interrupt
        util::bit_clear(LPC_TIM3->MCR, MR_int[MR_reg]);
      }
    }
  }
  return;
}
