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

#ifndef _SOFTWARE_PWM_H_
#define _SOFTWARE_PWM_H_

#include <pinmapping.h>

constexpr uint32_t PWM_MAX_SOFTWARE_CHANNELS = 20;
constexpr uint32_t PWM_MATCH_OFFSET = 2; // in ticks (default prescaler makes this microseconds)

void software_pwm_init(const uint32_t prescale, const uint32_t period);

struct SwPwmData{ // 14bytes double linked list node, (16 with packing)
  pin_t pin = P_NC;
  uint32_t value = 0;
  SwPwmData* next = nullptr;
  SwPwmData* prev = nullptr;
};

// Memory constrained insert sorted double linked list with unique identifier
template <std::size_t N>
struct SoftwarePwmTable {
  std::array<SwPwmData, N> swpwm_pin_table {}; //16*N bytes (320bytes @ 20 channels)
  SwPwmData* swpwm_table_head = nullptr;
  SwPwmData* swpwm_table_tail = nullptr;
  SwPwmData* swpwm_table_free = &swpwm_pin_table[0];
  std::size_t length = 0;
  static const std::size_t capacity = N;

  SoftwarePwmTable() {
    for(std::size_t i = 0; i < swpwm_pin_table.size() - 1; ++i) {
      swpwm_pin_table[i].next = &swpwm_pin_table[i + 1];
    }
  }

  void init(const uint32_t prescale, const uint32_t period, const uint32_t int_priority = 1) {
    // Setup timer for Timer3 Interrupt controlled PWM
    LPC_SC->PCONP |= 1 << 23;                 // power on timer3
    LPC_TIM3->PR = prescale;                  // match PWM hardware prescaler
    LPC_TIM3->MCR = util::bitset_value(0, 1); // Interrupt on MR0, reset on MR0
    LPC_TIM3->MR0 = period - 1;               // match PWM hardware period
    LPC_TIM3->TCR = util::bit_value(0);       // enable the timer

    NVIC_SetPriority(TIMER3_IRQn, NVIC_EncodePriority(0, int_priority, 0));
  }

  constexpr uint32_t size() {
    return length;
  }

  constexpr bool exists(const pin_t pin) {
    SwPwmData* it = nullptr;
    while((it = (it == nullptr ? swpwm_table_head : it->next))) {
      if(it->pin == pin) return true;
    }
    return false;
  }

  bool remove(const pin_t pin) {
    SwPwmData* it = nullptr;
    while((it = (it == nullptr ? swpwm_table_head : it->next))) {
      if(it->pin == pin) break;
    }

    if(it != nullptr) {
      NVIC_DisableIRQ(TIMER3_IRQn);
      __DSB();
      __ISB();
      if(it == swpwm_table_head) swpwm_table_head = swpwm_table_head->next;
      if(it == swpwm_table_tail) swpwm_table_tail = swpwm_table_tail->prev;
      if(it->next != nullptr) it->next->prev = it->prev;
      if(it->prev != nullptr) it->prev->next = it->next;
      it->prev = nullptr;
      it->next = swpwm_table_free;
      swpwm_table_free = it;
      length--;
      if(length) NVIC_EnableIRQ(TIMER3_IRQn);
      return true;
    }
    return false;
  }

  bool update(const pin_t pin, const uint32_t value) {
    SwPwmData* pos = nullptr;
    SwPwmData* cur = nullptr;
    SwPwmData* node = nullptr;

    // find the appropriate position and if it exists the currently used node for the pin
    for (SwPwmData* it = swpwm_table_head; it != nullptr; it = it->next) {
      if(it->pin == pin) cur = it;
      if(value < it->value && pos == nullptr) pos = it;
    }

      // Interupts disabled here
    NVIC_DisableIRQ(TIMER3_IRQn);
    __DSB();
    __ISB();

    // alocate new node
    if(pos != nullptr && pos == cur) {
      // change in place
      cur->value = value;
      NVIC_EnableIRQ(TIMER3_IRQn);
      return true;
    } else if(cur != nullptr) {
      // decouple current node for reinsertion
      if(cur == swpwm_table_head) swpwm_table_head = swpwm_table_head->next;
      if(cur == swpwm_table_tail) swpwm_table_tail = swpwm_table_tail->prev;
      if(cur->next != nullptr) cur->next->prev = cur->prev;
      if(cur->prev != nullptr) cur->prev->next = cur->next;
      node = cur;
    } else {
      // allocate new node from reserve
      node = swpwm_table_free;
      if(node == nullptr) {
        NVIC_EnableIRQ(TIMER3_IRQn);
        return false; // out of reserved memory
      }
      swpwm_table_free = node->next;
      length++;
    }

    // update node data
    node->pin = pin;
    node->value = value;

    if(pos == nullptr) {
      if(swpwm_table_head == nullptr) {
        // initialise head
        swpwm_table_head = node;
        swpwm_table_tail = node;
        node->next = nullptr;
        node->prev = nullptr;
      } else {
        // insert at tail
        swpwm_table_tail->next = node;
        node->prev = swpwm_table_tail;
        node->next = nullptr;
        swpwm_table_tail = node;
      }
    } else if(pos->prev == nullptr) {
      // insert at head
      node->next = swpwm_table_head;
      node->prev = nullptr;
      swpwm_table_head->prev = node;
      swpwm_table_head = node;
    } else {
      // insert in middle
      node->prev = pos->prev;
      node->next = pos;
      node->prev->next = node;
      pos->prev = node;
    }
    // Interupts enabled
    NVIC_EnableIRQ(TIMER3_IRQn);
    return true;
  }
};

extern SoftwarePwmTable<PWM_MAX_SOFTWARE_CHANNELS> SoftwarePWM;

#endif // _SOFTWARE_PWM_H_
