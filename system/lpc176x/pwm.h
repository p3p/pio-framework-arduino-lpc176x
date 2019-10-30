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

#ifndef _LPC1768_PWM_H_
#define _LPC1768_PWM_H_

#include <pinmapping.h>

void pwm_init(const uint32_t frequency = 50);
bool pwm_attach_pin(const pin_t pin, const uint32_t value = 0);
bool pwm_write(const pin_t pin, const uint32_t value);
bool pwm_write_ratio(const pin_t pin, const uint8_t value);
bool pwm_write_ratio(const pin_t pin, const float value);
bool pwm_write_us(const pin_t pin, const uint32_t value);
bool pwm_detach_pin(const pin_t pin);

uint32_t pwm_get_period(const pin_t pin);
bool pwm_set_frequency(const pin_t pin, const uint32_t frequency);

#endif // _LPC1768_PWM_H_
