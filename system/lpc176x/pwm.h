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

#pragma once

#include <pinmapping.h>

namespace LPC176x {

void pwm_init(const uint32_t frequency = 50);
bool pwm_attached(const pin_t pin);
bool pwm_attach_pin(const pin_t pin, const uint32_t value = 0, const bool force_sw = false);
bool pwm_write(const pin_t pin, const uint32_t value);
bool pwm_write_ratio(const pin_t pin, const uint8_t value);
bool pwm_write_ratio(const pin_t pin, const float value);
bool pwm_write_us(const pin_t pin, const uint32_t value);
bool pwm_detach_pin(const pin_t pin);

uint32_t pwm_get_period(const pin_t pin);
bool pwm_set_frequency(const pin_t pin, const uint32_t frequency, const bool force_sw = false);

} // LPC176x
