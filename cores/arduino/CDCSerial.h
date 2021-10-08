/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
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

#ifndef _HAL_SERIAL_H_
#define _HAL_SERIAL_H_

#include <stdarg.h>
#include <stdio.h>
#include <Stream.h>
#include <usb/usb.h>
#include <usb/usbcfg.h>
#include <usb/usbhw.h>
#include <usb/usbreg.h>
#include <usb/cdcuser.h>
#include <Arduino.h>

#ifndef USBCDCTIMEOUT
  #define USBCDCTIMEOUT 6
#endif

/**
 * Generic RingBuffer
 * T type of the buffer array
 * S size of the buffer (must be power of 2)
 */

template <typename T, std::size_t S> class RingBuffer {
public:
  RingBuffer() {index_read = index_write = 0;}

  std::size_t available() const {return mask(index_write - index_read);}
  std::size_t free() const {return size() - available();}
  bool empty() const {return index_read == index_write;}
  bool full() const {return next(index_write) == index_read;}
  void clear() {index_read = index_write = 0;}

  bool peek(T *const value) const {
    if (value == nullptr || empty()) return false;
    *value = buffer[index_read];
    return true;
  }

  [[gnu::always_inline, gnu::optimize("O3")]] inline std::size_t read(T* dst, std::size_t length) {
    length = std::min(length, available());
    const std::size_t length1 = std::min(length, buffer_size - index_read);
    memcpy(dst, (char*)buffer + index_read, length1);
    memcpy(dst + length1, (char*)buffer, length - length1);
    index_read = mask(index_read + length);
    return length;
  }

  [[gnu::always_inline, gnu::optimize("O3")]] inline std::size_t write(T* src, std::size_t length) {
    length = std::min(length, free());
    const std::size_t length1 = std::min(length, buffer_size - index_write);
    memcpy((char*)buffer + index_write, src, length1);
    memcpy((char*)buffer, src + length1, length - length1);
    index_write = mask(index_write + length);
    return length;
  }

  std::size_t read(T *const value) {
    if (value == nullptr || empty()) return 0;
    *value = buffer[index_read];
    index_read = next(index_read);
    return 1;
  }

  std::size_t write(const T value) {
    std::size_t next_head = next(index_write);
    if (next_head == index_read) return 0;     // buffer full
    buffer[index_write] = value;
    index_write = next_head;
    return 1;
  }

  constexpr std::size_t size() const {
    return buffer_size - 1;
  }

private:
  inline std::size_t mask(std::size_t val) const {
    return val & buffer_mask;
  }

  inline std::size_t next(std::size_t val) const {
    return mask(val + 1);
  }

  static const std::size_t buffer_size = S;
  static const std::size_t buffer_mask = buffer_size - 1;
  volatile T buffer[buffer_size];
  volatile std::size_t index_write;
  volatile std::size_t index_read;
};

/**
 *  Serial Interface Class
 *  Data is injected directly into, and consumed from, the fifo buffers
 */

class CDCSerial: public Stream {
public:

  CDCSerial() : host_connected(false) { }
  virtual ~CDCSerial() { }

  operator bool() { return host_connected; }

  void begin(int32_t baud) { }

  int16_t peek() {
    uint8_t value;
    return receive_buffer.peek(&value) ? value : -1;
  }

  int16_t read() {
    uint8_t value;
    uint32_t ret = receive_buffer.read(&value);
    CDC_FillBuffer(receive_buffer.free());
    return (ret ? value : -1);
  }

  size_t readBytes(uint8_t* dst, size_t length) {
    return readBytes(dst, length);
  }

  size_t readBytes(char* dst, size_t length) {
    size_t buffered = receive_buffer.read((uint8_t*)dst, length);
    const uint32_t usb_rx_timeout = millis() + USBCDCTIMEOUT;
    while (buffered != length && LPC176x::util::pending(millis(), usb_rx_timeout)) {
      if (!host_connected) return 0;
      CDC_FillBuffer(receive_buffer.free());
      buffered += receive_buffer.read((uint8_t*)dst + buffered, length - buffered);
    }
    CDC_FillBuffer(receive_buffer.free());
    return buffered;
  }

  size_t write(const char* src, size_t length) {
    size_t buffered = transmit_buffer.write((uint8_t*)src, length);
    const uint32_t usb_tx_timeout = millis() + USBCDCTIMEOUT;
    while (buffered != length && LPC176x::util::pending(millis(), usb_tx_timeout)) {
      if (!host_connected) return 0;
      buffered += transmit_buffer.write((uint8_t*)src + buffered, length - buffered);
      CDC_FlushBuffer();
    }
    CDC_FlushBuffer();
    return buffered;
  }

  size_t write(const uint8_t c) {
    if (!host_connected) return 0;          // Do not fill buffer when host disconnected
    const uint32_t usb_tx_timeout = millis() + USBCDCTIMEOUT;
    while (transmit_buffer.write(c) == 0 && LPC176x::util::pending(millis(), usb_tx_timeout)) { // Block until there is free room in buffer
      if (!host_connected) return 0;        // Break infinite loop on host disconect
      CDC_FlushBuffer();
    }
    CDC_FlushBuffer();
    return 1;
  }

  size_t available() {
    return receive_buffer.available();
  }

  // todo: change to conform to Arduino 1.0?
  void flush() {
    receive_buffer.clear();
    CDC_FillBuffer(receive_buffer.free());
  }

  uint8_t availableForWrite(void) {
    return min(transmit_buffer.free(), size_t(255));
  }

  void flushTX(void) {
    const uint32_t usb_tx_timeout = millis() + USBCDCTIMEOUT;
    while (transmit_buffer.available() && host_connected && LPC176x::util::pending(millis(), usb_tx_timeout)) {
      CDC_FlushBuffer();
    }
  }

  size_t printf(const char *format, ...) {
    static char buffer[256];
    va_list vArgs;
    va_start(vArgs, format);
    int length = vsnprintf((char *) buffer, 256, (char const *) format, vArgs);
    va_end(vArgs);
    size_t i = 0;
    if (length > 0 && length < 256) {
      uint32_t usb_tx_timeout = millis() + USBCDCTIMEOUT;
      while (i < (size_t)length && host_connected && LPC176x::util::pending(millis(), usb_tx_timeout)) {
        i += transmit_buffer.write(buffer[i]);
        CDC_FlushBuffer();
      }
    }
    return i;
  }

  RingBuffer<uint8_t, 128> receive_buffer;
  RingBuffer<uint8_t, 128> transmit_buffer;
  volatile bool host_connected;
};

extern CDCSerial UsbSerial;

#endif // _HAL_SERIAL_H_
