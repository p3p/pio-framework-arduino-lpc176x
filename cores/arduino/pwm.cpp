extern "C" {
  #include <LPC17xx.h>
}
#include "const_functions.h"
#include "pwm.h"

void HAL_pwm_init(void) {
  LPC_PINCON->PINSEL4 = _BV(PWM_5) | _BV(PWM_6);

  LPC_PWM1->TCR = _BV(SBIT_CNTEN) | _BV(SBIT_PWMEN);
  LPC_PWM1->PR  =  0x0;               // No prescalar
  LPC_PWM1->MCR = _BV(SBIT_PWMMR0R);  // Reset on PWMMR0, reset TC if it matches MR0
  LPC_PWM1->MR0 = 255;                // set PWM cycle(Ton+Toff)=255)
  LPC_PWM1->MR5 = 0;                  // Set 50% Duty Cycle for the channels
  LPC_PWM1->MR6 = 0;

  // Trigger the latch Enable Bits to load the new Match Values MR0, MR5, MR6
  LPC_PWM1->LER = _BV(0) | _BV(5) | _BV(6);
  // Enable the PWM output pins for PWM_5-PWM_6(P2_04 - P2_05)
  LPC_PWM1->PCR = _BV(13) | _BV(14);
}