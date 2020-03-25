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

#ifndef HardwareSerial_h
#define HardwareSerial_h

#include <array>
#include <stdarg.h>
#include <stdio.h>
#include <Stream.h>

extern "C" {
  #include <lpc17xx_uart.h>
  #include "lpc17xx_pinsel.h"
}

#if !defined(SERIAL_TX_BUFFER_SIZE)
  #define SERIAL_TX_BUFFER_SIZE 64
#endif
#if !defined(SERIAL_RX_BUFFER_SIZE)
  #define SERIAL_RX_BUFFER_SIZE 256
#endif

template <uint32_t RXB_SIZE = SERIAL_RX_BUFFER_SIZE, uint32_t TXB_SIZE = SERIAL_TX_BUFFER_SIZE>
class HardwareSerial : public Stream {
private:
  LPC_UART_TypeDef *UARTx;

  uint32_t Baudrate;
  uint32_t Status;
  std::array<uint8_t, RXB_SIZE> RxBuffer;
  uint32_t RxQueueWritePos;
  uint32_t RxQueueReadPos;
  std::array<uint8_t, TXB_SIZE> TxBuffer;
  uint32_t TxQueueWritePos;
  uint32_t TxQueueReadPos;

public:
  HardwareSerial(LPC_UART_TypeDef *UARTx)
    : UARTx(UARTx)
    , Baudrate(0)
    , RxQueueWritePos(0)
    , RxQueueReadPos(0)
    , TxQueueWritePos(0)
    , TxQueueReadPos(0)
  {
  }

  void begin(uint32_t baudrate) {
    UART_CFG_Type UARTConfigStruct;
    PINSEL_CFG_Type PinCfg;
    UART_FIFO_CFG_Type FIFOConfig;

    if (Baudrate == baudrate) return; // No need to re-initialize

    if (UARTx == LPC_UART0) {
      // Initialize UART0 pin connect
      PinCfg.Funcnum = 1;
      PinCfg.OpenDrain = 0;
      PinCfg.Pinmode = 0;
      PinCfg.Pinnum = 2;
      PinCfg.Portnum = 0;
      PINSEL_ConfigPin(&PinCfg);
      PinCfg.Pinnum = 3;
      PINSEL_ConfigPin(&PinCfg);
    } else if ((LPC_UART1_TypeDef *) UARTx == LPC_UART1) {
      // Initialize UART1 pin connect
      PinCfg.Funcnum = 1;
      PinCfg.OpenDrain = 0;
      PinCfg.Pinmode = 0;
      PinCfg.Pinnum = 15;
      PinCfg.Portnum = 0;
      PINSEL_ConfigPin(&PinCfg);
      PinCfg.Pinnum = 16;
      PINSEL_ConfigPin(&PinCfg);
    } else if (UARTx == LPC_UART2) {
      // Initialize UART2 pin connect
      PinCfg.Funcnum = 1;
      PinCfg.OpenDrain = 0;
      PinCfg.Pinmode = 0;
      PinCfg.Pinnum = 10;
      PinCfg.Portnum = 0;
      PINSEL_ConfigPin(&PinCfg);
      PinCfg.Pinnum = 11;
      PINSEL_ConfigPin(&PinCfg);
    } else if (UARTx == LPC_UART3) {
      // Initialize UART3 pin connect
      PinCfg.Funcnum = 3;
      PinCfg.OpenDrain = 0;
      PinCfg.Pinmode = 0;
      PinCfg.Pinnum = 28;
      PinCfg.Portnum = 4;
      PINSEL_ConfigPin(&PinCfg);
      PinCfg.Pinnum = 29;
      PINSEL_ConfigPin(&PinCfg);
    }

    /* Initialize UART Configuration parameter structure to default state:
     * Baudrate = 9600bps
     * 8 data bit
     * 1 Stop bit
     * None parity
     */
    UART_ConfigStructInit(&UARTConfigStruct);

    // Re-configure baudrate
    UARTConfigStruct.Baud_rate = baudrate;

    // Initialize eripheral with given to corresponding parameter
    UART_Init(UARTx, &UARTConfigStruct);

    // Enable and reset the TX and RX FIFOs
    UART_FIFOConfigStructInit(&FIFOConfig);
    UART_FIFOConfig(UARTx, &FIFOConfig);

    // Enable UART Transmit
    UART_TxCmd(UARTx, ENABLE);

    // Configure Interrupts
    UART_IntConfig(UARTx, UART_INTCFG_RBR, ENABLE);
    UART_IntConfig(UARTx, UART_INTCFG_RLS, ENABLE);

    // Set proper priority and enable interrupts
    if (UARTx == LPC_UART0) {
      NVIC_SetPriority(UART0_IRQn, NVIC_EncodePriority(0, 3, 0));
      NVIC_EnableIRQ(UART0_IRQn);
    }
    else if ((LPC_UART1_TypeDef *) UARTx == LPC_UART1) {
      NVIC_SetPriority(UART1_IRQn, NVIC_EncodePriority(0, 3, 0));
     NVIC_EnableIRQ(UART1_IRQn);
    }
    else if (UARTx == LPC_UART2) {
      NVIC_SetPriority(UART2_IRQn, NVIC_EncodePriority(0, 3, 0));
      NVIC_EnableIRQ(UART2_IRQn);
    }
    else if (UARTx == LPC_UART3) {
      NVIC_SetPriority(UART3_IRQn, NVIC_EncodePriority(0, 3, 0));
      NVIC_EnableIRQ(UART3_IRQn);
    }

    RxQueueWritePos = RxQueueReadPos = 0;
    if constexpr (TXB_SIZE > 0) {
      TxQueueWritePos = TxQueueReadPos = 0;
    }

    // Save the configured baudrate
    Baudrate = baudrate;
  }

  int16_t peek() {
    int16_t byte = -1;

    // Temporarily lock out UART receive interrupts during this read so the UART receive
    // interrupt won't cause problems with the index values
    UART_IntConfig(UARTx, UART_INTCFG_RBR, DISABLE);

    if (RxQueueReadPos != RxQueueWritePos)
      byte = RxBuffer[RxQueueReadPos];

    // Re-enable UART interrupts
    UART_IntConfig(UARTx, UART_INTCFG_RBR, ENABLE);

    return byte;
  }

  int16_t read() {
    int16_t byte = -1;

    // Temporarily lock out UART receive interrupts during this read so the UART receive
    // interrupt won't cause problems with the index values
    UART_IntConfig(UARTx, UART_INTCFG_RBR, DISABLE);

    if (RxQueueReadPos != RxQueueWritePos) {
      byte = RxBuffer[RxQueueReadPos];
      RxQueueReadPos = (RxQueueReadPos + 1) % RXB_SIZE;
    }

    // Re-enable UART interrupts
    UART_IntConfig(UARTx, UART_INTCFG_RBR, ENABLE);

    return byte;
  }

  size_t write(uint8_t send) {
    if constexpr (TXB_SIZE > 0) {
      size_t bytes = 0;
      uint32_t fifolvl = 0;

      // If the Tx Buffer is full, wait for space to clear
      if ((TxQueueWritePos+1) % TXB_SIZE == TxQueueReadPos) flushTX();

      // Temporarily lock out UART transmit interrupts during this read so the UART transmit interrupt won't
      // cause problems with the index values
      UART_IntConfig(UARTx, UART_INTCFG_THRE, DISABLE);

      // LPC17xx.h incorrectly defines FIFOLVL as a uint8_t, when it's actually a 32-bit register
      if ((LPC_UART1_TypeDef *) UARTx == LPC_UART1) {
        fifolvl = *(reinterpret_cast<volatile uint32_t *>(&((LPC_UART1_TypeDef *) UARTx)->FIFOLVL));
      } else fifolvl = *(reinterpret_cast<volatile uint32_t *>(&UARTx->FIFOLVL));

      // If the queue is empty and there's space in the FIFO, immediately send the byte
      if (TxQueueWritePos == TxQueueReadPos && fifolvl < UART_TX_FIFO_SIZE) {
        bytes = UART_Send(UARTx, &send, 1, BLOCKING);
      }
      // Otherwiise, write the byte to the transmit buffer
      else if ((TxQueueWritePos+1) % TXB_SIZE != TxQueueReadPos) {
        TxBuffer[TxQueueWritePos] = send;
        TxQueueWritePos = (TxQueueWritePos+1) % TXB_SIZE;
        bytes++;
      }

      // Re-enable the TX Interrupt
      UART_IntConfig(UARTx, UART_INTCFG_THRE, ENABLE);

      return bytes;
    } else {
      return UART_Send(UARTx, &send, 1, BLOCKING); // no tx buffer
    }
  }

  size_t write(char* src, size_t length) {
    for (size_t i = 0; i < length; ++i) {
      write(src[i]);
    }
    return length;
  }

  void flushTX() {
    if constexpr (TXB_SIZE > 0) {
      // Wait for the tx buffer and FIFO to drain
      while (TxQueueWritePos != TxQueueReadPos && UART_CheckBusy(UARTx) == SET);
    }
  }

  size_t available() {
    return (RxQueueWritePos + RXB_SIZE - RxQueueReadPos) % RXB_SIZE;
  }

  void flush() {
    RxQueueWritePos = 0;
    RxQueueReadPos = 0;
  }

  size_t printf(const char *format, ...) {
    char RxBuffer[256];
    va_list vArgs;
    va_start(vArgs, format);
    int length = vsnprintf(RxBuffer, 256, format, vArgs);
    va_end(vArgs);
    if (length > 0 && length < 256) {
      for (size_t i = 0; i < (size_t)length; ++i)
        write(RxBuffer[i]);
    }
    return length;
  }

  operator bool() { return true; }
  virtual bool recv_callback(const char c) { return true; }

  void IRQHandler() {
    uint32_t IIRValue;
    uint8_t LSRValue, byte;

    IIRValue = UART_GetIntId(UARTx);
    IIRValue &= UART_IIR_INTID_MASK; // check bit 1~3, interrupt identification

    // Receive Line Status
    if (IIRValue == UART_IIR_INTID_RLS) {
      LSRValue = UART_GetLineStatus(UARTx);

      // Receive Line Status
      if (LSRValue & (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE | UART_LSR_RXFE | UART_LSR_BI)) {
        // There are errors or break interrupt
        // Read LSR will clear the interrupt
        Status = LSRValue;
        byte = UART_ReceiveByte(UARTx); // Dummy read on RX to clear interrupt, then bail out
        return;
      }
    }

    // Receive Data Available
    if (IIRValue == UART_IIR_INTID_RDA) {
      // Clear the FIFO
      while (UART_Receive(UARTx, &byte, 1, NONE_BLOCKING)) {
        if(!recv_callback(byte)) continue;
        if ((RxQueueWritePos + 1) % RXB_SIZE != RxQueueReadPos) {
          RxBuffer[RxQueueWritePos] = byte;
          RxQueueWritePos = (RxQueueWritePos + 1) % RXB_SIZE;
        } else
          break;
      }
      // Character timeout indicator
    } else if (IIRValue == UART_IIR_INTID_CTI) {
      // Character Time-out indicator
      Status |= 0x100; // Bit 9 as the CTI error
    }

    if constexpr (TXB_SIZE > 0) {
      if (IIRValue == UART_IIR_INTID_THRE) {
        // Disable THRE interrupt
        UART_IntConfig(UARTx, UART_INTCFG_THRE, DISABLE);

        // Wait for FIFO buffer empty
        while (UART_CheckBusy(UARTx) == SET);

        // Transfer up to UART_TX_FIFO_SIZE bytes of data
        for (int i = 0; i < UART_TX_FIFO_SIZE && TxQueueWritePos != TxQueueReadPos; i++) {
          // Move a piece of data into the transmit FIFO
          if (UART_Send(UARTx, &TxBuffer[TxQueueReadPos], 1, NONE_BLOCKING)) {
            TxQueueReadPos = (TxQueueReadPos+1) % TXB_SIZE;
          } else break;
        }

        // If there is no more data to send, disable the transmit interrupt - else enable it or keep it enabled
        if (TxQueueWritePos == TxQueueReadPos) {
          UART_IntConfig(UARTx, UART_INTCFG_THRE, DISABLE);
        } else UART_IntConfig(UARTx, UART_INTCFG_THRE, ENABLE);
      }
    }
  }
};

extern  HardwareSerial<> Serial;
extern  HardwareSerial<> Serial1;
extern  HardwareSerial<> Serial2;
extern  HardwareSerial<> Serial3;

#endif // HardwareSerial_h
