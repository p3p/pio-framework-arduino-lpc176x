/*----------------------------------------------------------------------------
 *      U S B  -  K e r n e l
 *----------------------------------------------------------------------------
 *      Name:    cdcuser.c
 *      Purpose: USB Communication Device Class User module
 *      Version: V1.10
 *----------------------------------------------------------------------------
 *      This software is supplied "AS IS" without any warranties, express,
 *      implied or statutory, including but not limited to the implied
 *      warranties of fitness for purpose, satisfactory quality and
 *      noninfringement. Keil extends you a royalty-free right to reproduce
 *      and distribute executable files created using this software for use
 *      on NXP Semiconductors LPC microcontroller devices only. Nothing else
 *      gives you the right to use this software.
 *
 * Copyright (c) 2009 Keil - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

extern "C" {
  #include <LPC17xx.h>
  #include <lpc17xx_wdt.h>
  #include <lpc_types.h>
}

#include <usb/usb.h>
#include <usb/cdc.h>
#include <usb/usbcfg.h>
#include <usb/usbhw.h>
#include <usb/usbcore.h>
#include <usb/usbuser.h>
#include <usb/cdcuser.h>

#include <CDCSerial.h>
#include <debug_frmwrk.h>

unsigned char BulkBufIn[USB_CDC_BUFSIZE];            // Buffer to store USB IN  packet
unsigned char BulkBufOut[USB_CDC_BUFSIZE];            // Buffer to store USB OUT packet
unsigned char NotificationBuf[10];

CDC_LINE_CODING CDC_LineCoding = { 921600, 0, 0, 8 };
unsigned short CDC_LineState = 0;
unsigned short CDC_SerialState = 0;
volatile uint32_t CDC_OutAvailable = 0;
uint32_t CDC_OutOffset = 0;
bool InBusy = false;
bool OutBusy = false;
volatile bool InActive = false;
volatile bool OutActive = false;

extern CDCSerial UsbSerial;

__attribute__((weak)) bool CDC_RecvCallback(const char byte) {
  return true;
}

static void CDC_QueueDMAIO(uint32_t EPNum, uint8_t *pData, uint32_t cnt) {
  //_DBG("DMA IO "); _DBD32((uint32_t) pData); _DBG(" "); _DBD32(cnt); _DBG("\n");
  USB_DMA_DESCRIPTOR desc;
  desc.BufAdr = (uint32_t) pData;
  desc.BufLen = cnt;
  desc.MaxSize = (cnt > 0 ? USB_CDC_BUFSIZE : 0);
  desc.InfoAdr = 0;
  desc.Cfg.Val = 0;
  USB_DMA_Setup(EPNum, &desc);
}


/*----------------------------------------------------------------------------
 write data to CDC_OutBuf
 *---------------------------------------------------------------------------*/
void CDC_WrOutBuf(uint8_t *buffer, uint32_t bytesToWrite) {
  while (bytesToWrite--) {
    if(CDC_RecvCallback(*buffer)) {
      if (!UsbSerial.receive_buffer.write(*buffer++))
        _DBG("Overflow\n");
    }
  }
}

/* end Buffer handling */

/*----------------------------------------------------------------------------
 CDC Initialisation
 Initializes the data structures and serial port
 Parameters:   None
 Return Value: None
 *---------------------------------------------------------------------------*/
void CDC_Init() {
  CDC_LineState = 0;
  CDC_SerialState = 0;
  CDC_OutAvailable = 0;
  CDC_OutOffset = 0;
  InBusy = false;
  OutBusy = false;
  InActive = false;
  OutActive = false;
  UsbSerial.host_connected = false;
}

/*----------------------------------------------------------------------------
 CDC Suspend Event
 Handle the suspension of the USB connection
 Parameters:   None
 Return Value: None
 *---------------------------------------------------------------------------*/
void CDC_Suspend() {
  UsbSerial.host_connected = false;
  UsbSerial.transmit_buffer.clear();
}

/*----------------------------------------------------------------------------
 CDC Resume Event
 Handle the resumption of the USB connection
 Parameters:   None
 Return Value: None
 *---------------------------------------------------------------------------*/
void CDC_Resume() {
  UsbSerial.host_connected = (CDC_LineState & CDC_DTE_PRESENT) != 0 ? true : false;
}

/*----------------------------------------------------------------------------
 CDC Reset Event
 Handle the reset of the USB connection
 Parameters:   None
 Return Value: None
 *---------------------------------------------------------------------------*/
void CDC_Reset() {
  // USB reset, any packets in transit may have been flushed
  UsbSerial.host_connected = (CDC_LineState & CDC_DTE_PRESENT) != 0 ? true : false;
  CDC_OutAvailable = 0;
  CDC_OutOffset = 0;
  InBusy = false;
  OutBusy = false;
  InActive = false;
  OutActive = false;
  USB_DMA_Enable(CDC_DEP_OUT);
}

/*----------------------------------------------------------------------------
 CDC SendEncapsulatedCommand Request Callback
 Called automatically on CDC SEND_ENCAPSULATED_COMMAND Request
 Parameters:   None                          (global SetupPacket and EP0Buf)
 Return Value: TRUE - Success, FALSE - Error
 *---------------------------------------------------------------------------*/
uint32_t CDC_SendEncapsulatedCommand(void) {

  return (TRUE);
}

/*----------------------------------------------------------------------------
 CDC GetEncapsulatedResponse Request Callback
 Called automatically on CDC Get_ENCAPSULATED_RESPONSE Request
 Parameters:   None                          (global SetupPacket and EP0Buf)
 Return Value: TRUE - Success, FALSE - Error
 *---------------------------------------------------------------------------*/
uint32_t CDC_GetEncapsulatedResponse(void) {

  /* ... add code to handle request */
  return (TRUE);
}

/*----------------------------------------------------------------------------
 CDC SetCommFeature Request Callback
 Called automatically on CDC Set_COMM_FATURE Request
 Parameters:   FeatureSelector
 Return Value: TRUE - Success, FALSE - Error
 *---------------------------------------------------------------------------*/
uint32_t CDC_SetCommFeature(unsigned short wFeatureSelector) {

  /* ... add code to handle request */
  return (TRUE);
}

/*----------------------------------------------------------------------------
 CDC GetCommFeature Request Callback
 Called automatically on CDC Get_COMM_FATURE Request
 Parameters:   FeatureSelector
 Return Value: TRUE - Success, FALSE - Error
 *---------------------------------------------------------------------------*/
uint32_t CDC_GetCommFeature(unsigned short wFeatureSelector) {

  /* ... add code to handle request */
  return (TRUE);
}

/*----------------------------------------------------------------------------
 CDC ClearCommFeature Request Callback
 Called automatically on CDC CLEAR_COMM_FATURE Request
 Parameters:   FeatureSelector
 Return Value: TRUE - Success, FALSE - Error
 *---------------------------------------------------------------------------*/
uint32_t CDC_ClearCommFeature(unsigned short wFeatureSelector) {

  /* ... add code to handle request */
  return (TRUE);
}

/*----------------------------------------------------------------------------
 CDC SetLineCoding Request Callback
 Called automatically on CDC SET_LINE_CODING Request
 Parameters:   none                    (global SetupPacket and EP0Buf)
 Return Value: TRUE - Success, FALSE - Error
 *---------------------------------------------------------------------------*/
uint32_t CDC_SetLineCoding(void) {

  CDC_LineCoding.dwDTERate = (EP0Buf[0] << 0) | (EP0Buf[1] << 8) | (EP0Buf[2] << 16) | (EP0Buf[3] << 24);
  CDC_LineCoding.bCharFormat = EP0Buf[4];
  CDC_LineCoding.bParityType = EP0Buf[5];
  CDC_LineCoding.bDataBits = EP0Buf[6];

  return (TRUE);
}

/*----------------------------------------------------------------------------
 CDC GetLineCoding Request Callback
 Called automatically on CDC GET_LINE_CODING Request
 Parameters:   None                         (global SetupPacket and EP0Buf)
 Return Value: TRUE - Success, FALSE - Error
 *---------------------------------------------------------------------------*/
uint32_t CDC_GetLineCoding(void) {

  EP0Buf[0] = (CDC_LineCoding.dwDTERate >> 0) & 0xFF;
  EP0Buf[1] = (CDC_LineCoding.dwDTERate >> 8) & 0xFF;
  EP0Buf[2] = (CDC_LineCoding.dwDTERate >> 16) & 0xFF;
  EP0Buf[3] = (CDC_LineCoding.dwDTERate >> 24) & 0xFF;
  EP0Buf[4] = CDC_LineCoding.bCharFormat;
  EP0Buf[5] = CDC_LineCoding.bParityType;
  EP0Buf[6] = CDC_LineCoding.bDataBits;

  return (TRUE);
}

/*----------------------------------------------------------------------------
 CDC SetControlLineState Request Callback
 Called automatically on CDC SET_CONTROL_LINE_STATE Request
 Parameters:   ControlSignalBitmap
 Return Value: TRUE - Success, FALSE - Error
 *---------------------------------------------------------------------------*/
uint32_t CDC_SetControlLineState(unsigned short wControlSignalBitmap) {
  CDC_LineState = wControlSignalBitmap;
  UsbSerial.host_connected = (CDC_LineState & CDC_DTE_PRESENT) != 0 ? true : false;
  return true;
}

/*----------------------------------------------------------------------------
 CDC SendBreak Request Callback
 Called automatically on CDC Set_COMM_FATURE Request
 Parameters:   0xFFFF  start of Break
 0x0000  stop  of Break
 0x####  Duration of Break
 Return Value: TRUE - Success, FALSE - Error
 *---------------------------------------------------------------------------*/
uint32_t CDC_SendBreak(unsigned short wDurationOfBreak) {

  /* ... add code to handle request */
  return (TRUE);
}

/*----------------------------------------------------------------------------
 CDC_BulkIn call on DataIn Request
 Parameters:   none
 Return Value: none
 *---------------------------------------------------------------------------*/
void CDC_BulkIn(void) {
  uint32_t numBytesAvail = UsbSerial.transmit_buffer.available();
  if (InBusy) _DBG("In Buffer busy\n");
  if (!InBusy && numBytesAvail > 0) {
    numBytesAvail = numBytesAvail > (USB_CDC_BUFSIZE - 1) ? (USB_CDC_BUFSIZE - 1) : numBytesAvail;
    for(uint32_t i = 0; i < numBytesAvail; ++i) {
      UsbSerial.transmit_buffer.read(&BulkBufIn[i]);
    }
    //if (numBytesAvail > 1) {
    //_DBG("Send "); _DBD32(numBytesAvail); _DBG("\n");
    //}
    InBusy = true;
    InActive = true;
    CDC_QueueDMAIO(CDC_DEP_IN, &BulkBufIn[0], numBytesAvail);
  }
  else
    InActive = false;
}

/*----------------------------------------------------------------------------
 CDC_BulkOut call on DataOut Request
 Parameters:   none
 Return Value: none
 *---------------------------------------------------------------------------*/
void CDC_BulkOut(void) {
  CDC_OutOffset = 0;
  CDC_OutAvailable = USB_DMA_BufCnt(CDC_DEP_OUT);
  //_DBG("Rec "); _DBD32(CDC_OutAvailable); _DBG("\n");
  CDC_WrOutBuf(&BulkBufOut[0], CDC_OutAvailable);
  OutBusy = false;
}

/*----------------------------------------------------------------------------
 Get the SERIAL_STATE as defined in usbcdc11.pdf, 6.3.5, Table 69.
 Parameters:   none
 Return Value: SerialState as defined in usbcdc11.pdf
 *---------------------------------------------------------------------------*/
unsigned short CDC_GetSerialState(void) {
  CDC_SerialState = CDC_LineState;
  //todo: detect buffer overrun
  return (CDC_SerialState);
}

/*----------------------------------------------------------------------------
 Send the SERIAL_STATE notification as defined in usbcdc11.pdf, 6.3.5.
 *---------------------------------------------------------------------------*/
void CDC_NotificationIn(void) {

  NotificationBuf[0] = 0xA1;                           // bmRequestType
  NotificationBuf[1] = CDC_NOTIFICATION_SERIAL_STATE;     // bNotification (SERIAL_STATE)
  NotificationBuf[2] = 0x00;                           // wValue
  NotificationBuf[3] = 0x00;
  NotificationBuf[4] = 0x00;                           // wIndex (Interface #, LSB first)
  NotificationBuf[5] = 0x00;
  NotificationBuf[6] = 0x02;                           // wLength (Data length = 2 bytes, LSB first)
  NotificationBuf[7] = 0x00;
  NotificationBuf[8] = (CDC_SerialState >> 0) & 0xFF;     // UART State Bitmap (16bits, LSB first)
  NotificationBuf[9] = (CDC_SerialState >> 8) & 0xFF;

  USB_WriteEP(CDC_CEP_IN, &NotificationBuf[0], 10);     // send notification
}

void CDC_DMA (uint32_t event) {
  //_DBG("DMA event "); _DBD32(event); _DBG(" "); _DBD32(USB_DMA_Status(MSC_EP_IN)); _DBG("\n");

  switch(event) {
    case USB_EVT_IN_DMA_EOT:
      InBusy = false;
      break;
    case USB_EVT_IN_DMA_NDR:
      CDC_BulkIn();
      break;
    case USB_EVT_IN_DMA_ERR:
      _DBG("Error in "); _DBD32(USB_DMA_Status(CDC_DEP_IN)); _DBG("\n");
      break;
    case USB_EVT_OUT_DMA_EOT:
      CDC_BulkOut();
      break;
    case USB_EVT_OUT_DMA_NDR:
      if (OutBusy) _DBG("Out buffer busy\n");
      if (!OutBusy && UsbSerial.receive_buffer.free() >= USB_CDC_BUFSIZE) {
        OutBusy = true;
        OutActive = true;
        CDC_QueueDMAIO(CDC_DEP_OUT, &BulkBufOut[0], USB_CDC_BUFSIZE);
      }
      else
        OutActive = false;
      break;
    case USB_EVT_OUT_DMA_ERR:
      _DBG("Error out "); _DBD32(USB_DMA_Status(CDC_DEP_OUT)); _DBG("\n");
      break;
  }
}
