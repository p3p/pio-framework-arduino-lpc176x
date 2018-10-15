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

#include "HardwarePWM.h"

uint32_t active_pwm_pins = 0;

void pwm_hardware_init(const uint32_t prescale, const uint32_t period) {
  // Reset and set up timing
  LPC_PWM1->TCR  = util::bit_value(1);  // reset and hold timer
  LPC_PWM1->PR   = prescale;
  LPC_PWM1->MR0  = 1000; //todo: if the period is set correctly here then the first set of match registers doesnt work, figure out why
  LPC_PWM1->MR1  = 0;
  LPC_PWM1->MR2  = 0;
  LPC_PWM1->MR3  = 0;
  LPC_PWM1->MR4  = 0;
  LPC_PWM1->MR5  = 0;
  LPC_PWM1->MR6  = 0;
  // Configure PWM
  LPC_PWM1->MCR  = util::bit_value(1);  // Configured to reset TC if it matches MR0, No interrupts
  LPC_PWM1->CTCR = 0;                   // Set counters to increment on prescaled timer
  // Disable all PWM outputs and enable PWM mode
  LPC_PWM1->PCR  = 0;                             // PWM1 control of outputs off
  LPC_PWM1->TCR  = util::bitset_value(0, 3);      // Enable counters, Turn on PWM
  pwm_set_period(period);                         // set and latch in period
}
