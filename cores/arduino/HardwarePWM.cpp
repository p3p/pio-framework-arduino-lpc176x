/**
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <lpc17xx_clkpwr.h>
#include "HardwarePWM.h"
uint32_t active_pwm_pins = 0;
uint32_t idle_pwm_pins = 0;

void pwm_hardware_init(const uint32_t prescale, const uint32_t period) {

  // Power on the peripheral
  CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCPWM1, ENABLE);
  CLKPWR_SetPCLKDiv (CLKPWR_PCLKSEL_PWM1, CLKPWR_PCLKSEL_CCLK_DIV_4);

  // Make sure it is in a clean state
  LPC_PWM1->IR = 0xFF & PWM_IR_BITMASK;
  LPC_PWM1->TCR = 0;
  LPC_PWM1->CTCR = 0;
  LPC_PWM1->MCR = 0;
  LPC_PWM1->CCR = 0;
  LPC_PWM1->PCR &= 0xFF00;
  LPC_PWM1->LER = 0;

  // Clock prescaler
  LPC_PWM1->PR = prescale;

  // Configured to reset TC if it matches MR0, No interrupts
  LPC_PWM1->MCR = util::bit_value(1);

  // Set the period using channel 0 before enabling peripheral
  pwm_set_period(period);

  // Enable PWM mode
  // TODO: this is very unreliable appears to randomly miss latches thus not changing the duty cycle
  // disabling PWM latch mode at least gives reliable (bit 3)
  //LPC_PWM1->TCR = util::bitset_value(0, 3);      //  Turn on PWM latch mode and Enable counters
  LPC_PWM1->TCR = util::bitset_value(0);
}
