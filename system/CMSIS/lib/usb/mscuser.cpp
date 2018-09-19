/*----------------------------------------------------------------------------
 *      U S B  -  K e r n e l
 *----------------------------------------------------------------------------
 *      Name:    MSCUSER.C
 *      Purpose: Mass Storage Class Custom User Module
 *      Version: V1.10
 *----------------------------------------------------------------------------
 *      This software is supplied "AS IS" without any warranties, express,
 *      implied or statutory, including but not limited to the implied
 *      warranties of fitness for purpose, satisfactory quality and
 *      noninfringement. Keil extends you a royalty-free right to reproduce
 *      and distribute executable files created using this software for use
 *      on NXP Semiconductors LPC family microcontroller devices only. Nothing
 *      else gives you the right to use this software.
 *
 *      Copyright (c) 2005-2009 Keil Software.
 *---------------------------------------------------------------------------*/

extern "C" {
  #include <LPC17xx.h>
  #include <lpc17xx_wdt.h>
  #include <lpc_types.h>
  #include <debug_frmwrk.h>
}

#include <usb/usb.h>
#include <usb/msc.h>
#include <usb/usbcfg.h>
#include <usb/usbhw.h>
#include <usb/usbcore.h>
#include <usb/mscuser.h>

#include <chanfs/diskio.h>

DWORD MSC_BlockCount = 0;

uint32_t MemOK;                   /* Memory OK */

DWORD lba;                        /* start block */
DWORD transfer_count;             /* blocks to transfer */
DWORD length;
uint32_t block_offset;            /* current block offset*/

uint8_t  BulkStage;               /* Bulk Stage */

uint8_t  BulkBuf[MSC_MAX_PACKET]; /* Bulk In/Out Buffer */
uint8_t  block_cache[MSC_BLOCK_SIZE];
uint8_t  BulkLen;                 /* Bulk In/Out Length */
Sense sense_data;

#define CMD_NONE   0
#define CMD_START  1
#define CMD_READ   2
#define CMD_WRITE  3
volatile uint8_t deferred_cmd = 0;
volatile uint32_t missed_ops = 0;
#define CACHE_INVALID  0xffffffff
volatile DWORD cache_lba = CACHE_INVALID;

MSC_CBW CBW;                   /* Command Block Wrapper */
MSC_CSW CSW;                   /* Command Status Wrapper */
volatile uint8_t media_lock = 0;
volatile bool device_wants_lock = false;
volatile bool ep_in_stalled = false;

#define NO_LOCK 0
#define HOST_LOCK 1
#define DEVICE_LOCK 2

extern uint32_t millis();
extern void _delay_ms(int delay);
void MSC_RunDeferredCommands();

uint32_t MSC_Aquire_Lock() {
  if (media_lock == DEVICE_LOCK) return 0;

  NVIC_DisableIRQ(USB_IRQn);
  device_wants_lock = true;
  uint32_t end_millis = millis() + 1000;
  if(media_lock == HOST_LOCK) {
    NVIC_EnableIRQ(USB_IRQn);
    while(media_lock == HOST_LOCK) {
      MSC_RunDeferredCommands();
      if(((long)(end_millis - (millis())) < 0)) {
        _DBG("No signal from Host, Assume success\n");
        break;
      }
      WDT_Feed();
    }
  }
  NVIC_DisableIRQ(USB_IRQn);
  media_lock = DEVICE_LOCK;
  NVIC_EnableIRQ(USB_IRQn);
  _DBG("Device MSC Lock\n");
  device_wants_lock = false;
  return 0;
}

uint32_t MSC_Release_Lock() {
  if(media_lock != DEVICE_LOCK) {
    return 0; // Didn't have lock
  }
  media_lock = NO_LOCK;
  if(disk_status(0) != STA_NOINIT) disk_ioctl(0, GET_SECTOR_COUNT, (void *)(&MSC_BlockCount));
  _DBG("Device MSC Unlock\n");
  NVIC_DisableIRQ(USB_IRQn);
  sense_data.set(Sense_KEY::UNIT_ATTENTION, Sense_ASC::MEDIA_CHANGED);
  NVIC_EnableIRQ(USB_IRQn);
  return 0;   // Released
}

uint32_t MSC_SD_Lock() {
  if(media_lock == DEVICE_LOCK || (device_wants_lock && CBW.CB[4])) {
    CSW.bStatus = CSW_CMD_FAILED;
    sense_data.set(Sense_KEY::NOT_READY, Sense_ASC::LOGICAL_UNIT_NOT_READY, Sense_ASCQ::DEVICE_IS_BUSY);
    MSC_SetCSW();
    _DBG("Device has Lock (or is waiting for lock) cannot Lock..\n");
    return 1;
  }

  if(CBW.CB[4]) {
    media_lock = HOST_LOCK;
    _DBG("OS MSC Lock\n");
  } else {
    media_lock = NO_LOCK;
    _DBG("OS MSC Unlock\n");
  }
  // logical_unit = CBW.CB[1] & 0xE0;
  CSW.bStatus = CSW_CMD_PASSED;
  MSC_SetCSW();
  return 0;
}

uint32_t MSC_SD_Release(uint8_t pdrv) {
  MSC_BlockCount = 0;
  deferred_cmd = CMD_NONE;
  return 0;
}

uint32_t MSC_SD_Init(uint8_t pdrv) {
  DSTATUS ret = disk_initialize(pdrv);
  if(ret) return ret;
  if(disk_ioctl (pdrv, GET_SECTOR_COUNT, (void *)(&MSC_BlockCount))) return 1;
  return 0;
}

void MSC_Start() {
  if(MSC_SD_Init(0) != 0) {
    CSW.bStatus = CSW_CMD_FAILED;
    sense_data.set(Sense_KEY::NOT_READY, Sense_ASC::MEDIUM_NOT_PRESENT, Sense_ASCQ::MANUAL_INTERVENTION_REQUIRED);
    _DBG("Disk init failed\n");
  }
  else {
    CSW.bStatus = CSW_CMD_PASSED;
    sense_data.reset();
    media_lock = HOST_LOCK;
    _DBG("Disk init worked\n");
  }
  MSC_SetCSW();
}

void MSC_DeferCommand(uint8_t cmd) {
  deferred_cmd = cmd;
}

#define STARTSTOP_STOPMOTOR        0x0
#define STARTSTOP_STARTMOTOR       0x1
#define STARTSTOP_EJECT            0x2
#define STARTSTOP_LOAD             0x3

void MSC_StartStopUnit() {
  switch (CBW.CB[4] & 0x03) {
    case STARTSTOP_EJECT:
      MSC_SD_Release(0);
      media_lock = NO_LOCK;
      _DBG("OS Media Ejected UNLOCK\n");
      break;
    case STARTSTOP_LOAD:
      if(MSC_BlockCount == 0) {
        MSC_DeferCommand(CMD_START);
        return;
      }
      media_lock = HOST_LOCK;
      _DBG("OS Media Mount LOCKED\n");
      break;
    default:
      _DBG("MSC_StartStopUnit unknown startstopunit sub command: ");
      _DBH(CBW.CB[4] & 0x03);
      _DBG("\n");
  }
  CSW.bStatus = CSW_CMD_PASSED;
  sense_data.reset();
  MSC_SetCSW();
}

/*
 *  MSC Mass Storage Reset Request Callback
 *   Called automatically on Mass Storage Reset Request
 *    Parameters:      None (global SetupPacket and EP0Buf)
 *    Return Value:    TRUE - Success, FALSE - Error
 */

uint32_t MSC_Reset (void) {
  BulkStage = MSC_BS_CBW;
  ep_in_stalled = false;
  return (TRUE);
}

/*
 *  MSC Mass Storage Unstall request callback
 *   Called when one of the msc bulk endpoints is unstalled
 *    Parameters:      EPNum the endpoint that is being unstalled
 *    Return Value:    None
 */

void MSC_UnstallEP(uint32_t EPNum) {
  if ((EPNum == MSC_EP_IN) && ep_in_stalled) {
    /* Compliance Test: rewrite CSW after unstall */
    if (CSW.dSignature == MSC_CSW_Signature) {
      USB_WriteEP(MSC_EP_IN, (uint8_t *) &CSW, sizeof(CSW));
    }
    ep_in_stalled = false;
  }
}

void MSC_StallEP(uint32_t EPNum) {
  USB_SetStallEP(EPNum);
  // keep track of the stall state of the IN EP.
  if (EPNum == MSC_EP_IN)
    ep_in_stalled = true;
}

/*
 *  MSC Get Max LUN Request Callback
 *   Called automatically on Get Max LUN Request
 *    Parameters:      None (global SetupPacket and EP0Buf)
 *    Return Value:    TRUE - Success, FALSE - Error
 */

uint32_t MSC_GetMaxLUN (void) {

  EP0Buf[0] = 0;               /* No LUN associated with this device */
  return (TRUE);
}

bool MSC_CheckAvailable(void) {
  if (MSC_BlockCount == 0)
    sense_data.set(Sense_KEY::NOT_READY, Sense_ASC::MEDIUM_NOT_PRESENT, Sense_ASCQ::LOADABLE);
  else if (media_lock == DEVICE_LOCK)
    sense_data.set(Sense_KEY::NOT_READY, Sense_ASC::MEDIUM_NOT_PRESENT, Sense_ASCQ::REASON_UNKNOWN);
  else if (device_wants_lock) {
    sense_data.set(Sense_KEY::NOT_READY, Sense_ASC::MEDIUM_NOT_PRESENT, Sense_ASCQ::REASON_UNKNOWN);
    // indicate the device can have the lock
    media_lock = NO_LOCK;
  }
  else
    return true;
  CSW.bStatus = CSW_CMD_FAILED;
  MSC_SetCSW();
  return false;
}

bool host_get_lock(void) {
  if (MSC_CheckAvailable()) {
    media_lock = HOST_LOCK;
    return true;
  }
  return false;
}

static void MSC_CompleteRead (void) {
  uint32_t n = (length > MSC_MAX_PACKET) ? MSC_MAX_PACKET : length;
  USB_WriteEP(MSC_EP_IN, &block_cache[block_offset], n);

  block_offset += n;
  length -= n;
  CSW.dDataResidue -= n;

  if(block_offset >= MSC_BLOCK_SIZE) {
    block_offset = 0;
    ++lba;
  }

  if (length == 0) {
    BulkStage = MSC_BS_DATA_IN_LAST;
  }

  if (BulkStage != MSC_BS_DATA_IN) {
    CSW.bStatus = CSW_CMD_PASSED;
    sense_data.reset();
  }
}

/*
 *  MSC Memory Read Callback
 *   Called automatically on Memory Read Event
 *    Parameters:      None (global variables)
 *    Return Value:    None
 */
void MSC_MemoryRead (void) {
  if (deferred_cmd != CMD_NONE){
    return;
  }
  if(!host_get_lock()) {
     return;
  }
  WDT_Feed();

  if (lba > MSC_BlockCount) {
    _DBG("Invalid LBA\n");
  }

  if(lba != cache_lba) {
    MSC_DeferCommand(CMD_READ);
    return;
  }
  MSC_CompleteRead();
}


/*
 * Complete the write operation. May be called folowing a
 * deferred operation.
 */
static void MSC_CompleteWrite(void) {
  block_offset += BulkLen;
  length -= BulkLen;
  CSW.dDataResidue -= BulkLen;

  if(block_offset >= MSC_BLOCK_SIZE) {
    block_offset = 0;
    ++lba;
  }

  if ((length == 0) || (BulkStage == MSC_BS_CSW)) {
    CSW.bStatus = CSW_CMD_PASSED;
    sense_data.reset();
    MSC_SetCSW();
  }
}

/*
 *  MSC Memory Write Callback
 *   Called automatically on Memory Write Event
 *    Parameters:      None (global variables)
 *    Return Value:    None
 */

void MSC_MemoryWrite (void) {
  if (deferred_cmd != CMD_NONE) {
    missed_ops++;
    return;
  }
  BulkLen = (uint8_t)USB_ReadEP(MSC_EP_OUT, BulkBuf);
  if(!host_get_lock()) {
     return;
  }
  WDT_Feed();
  for (uint32_t n = 0; n < BulkLen; n++) {
    block_cache[block_offset + n] = BulkBuf[n];
  }

  if(block_offset + BulkLen >= MSC_BLOCK_SIZE) {
    cache_lba = lba;
    if(!(disk_status(0) & STA_PROTECT)){
      MSC_DeferCommand(CMD_WRITE);
      return;
    }
  }
  MSC_CompleteWrite();
}


/*
 *  MSC Memory Verify Callback
 *   Called automatically on Memory Verify Event
 *    Parameters:      None (global variables)
 *    Return Value:    None
 */

void MSC_MemoryVerify (void) {
  BulkLen = (uint8_t)USB_ReadEP(MSC_EP_OUT, BulkBuf);
  if(!host_get_lock()) {
     return;
  }
  WDT_Feed();


  block_offset += BulkLen;
  length -= BulkLen;
  CSW.dDataResidue -= BulkLen;

  if ((length == 0) || (BulkStage == MSC_BS_CSW)) {
    if(MemOK) {
      CSW.bStatus = CSW_CMD_PASSED;
      sense_data.reset();
    } else {
      CSW.bStatus = CSW_CMD_FAILED;
      sense_data.set(Sense_KEY::MEDIUM_ERROR);
    }
    MSC_SetCSW();
  }
}

/*
 * MSC_RunDeferredCommands
 * Called to allow comands that access shared hardware to run outside
 * of an interrupt context. hould be called from "main loop" of the
 * user program.
 */
void MSC_RunDeferredCommands() {
  uint8_t cmd = deferred_cmd;
  if (cmd == CMD_NONE) return;
  NVIC_DisableIRQ(USB_IRQn);
  deferred_cmd = CMD_NONE;
  switch(cmd) {
    case CMD_NONE:
      break;
    case CMD_START:
      MSC_Start();
      break;
    case CMD_READ:
      disk_read (0, block_cache, lba, 1);
      cache_lba = lba;
      MSC_CompleteRead();
      break;
    case CMD_WRITE:
      disk_write(0, block_cache, cache_lba, 1);
      MSC_CompleteWrite();
      if (missed_ops > 0) {
        MSC_MemoryWrite();
        missed_ops = 0;
      }
      break;
  }
  NVIC_EnableIRQ(USB_IRQn);
}

/*
 *  MSC SCSI Read/Write Setup Callback
 *    Parameters:      None (global variables)
 *    Return Value:    TRUE - Success, FALSE - Error
 */

uint32_t MSC_RWSetup (void) {
  uint32_t n;

  /* Logical Block Address of First Block */
  lba = (CBW.CB[2] << 24) |
        (CBW.CB[3] << 16) |
        (CBW.CB[4] <<  8) |
        (CBW.CB[5] <<  0);

  /* Number of Blocks to transfer */
  transfer_count = (CBW.CB[7] <<  8) |
                   (CBW.CB[8] <<  0);

  block_offset = 0;
  length = transfer_count * MSC_BLOCK_SIZE;

  if (CBW.dDataLength != (transfer_count * MSC_BLOCK_SIZE)) {
    CSW.bStatus = CSW_PHASE_ERROR;
    MSC_SetCSW();
    return (FALSE);
  }
  return (TRUE);
}


/*
 *  Check Data IN Format
 *    Parameters:      None (global variables)
 *    Return Value:    TRUE - Success, FALSE - Error
 */

uint32_t DataInFormat (void) {

  if (CBW.dDataLength == 0) {
    CSW.bStatus = CSW_PHASE_ERROR;
    MSC_SetCSW();
    return (FALSE);
  }
  if ((CBW.bmFlags & 0x80) == 0) {
    CSW.bStatus = CSW_PHASE_ERROR;
    MSC_SetCSW();
    return (FALSE);
  }
  return (TRUE);
}


/*
 *  Perform Data IN Transfer
 *    Parameters:      None (global variables)
 *    Return Value:    TRUE - Success, FALSE - Error
 */

void DataInTransfer (void) {
  if (BulkLen > CBW.dDataLength) {
    BulkLen = CBW.dDataLength;
  }

  USB_WriteEP(MSC_EP_IN, BulkBuf, BulkLen);
  BulkStage = MSC_BS_DATA_IN_LAST;

  CSW.dDataResidue -= BulkLen;
  CSW.bStatus = CSW_CMD_PASSED;
}


/*
 *  MSC SCSI Test Unit Ready Callback
 *    Parameters:      None (global variables)
 *    Return Value:    None
 */

void MSC_TestUnitReady (void) {

  if (MSC_CheckAvailable()) {
    sense_data.reset();
    CSW.bStatus = CSW_CMD_PASSED;
    MSC_SetCSW();
  }
}


/*
 *  MSC SCSI Request Sense Callback
 *    Parameters:      None (global variables)
 *    Return Value:    None
 */

void MSC_RequestSense (void) {

  if (!DataInFormat()) return;

  BulkBuf[ 0] = 0x70;          /* Response Code */
  BulkBuf[ 1] = 0x00;
  BulkBuf[ 2] = static_cast<uint8_t>(sense_data.key);
  BulkBuf[ 3] = 0x00;
  BulkBuf[ 4] = 0x00;
  BulkBuf[ 5] = 0x00;
  BulkBuf[ 6] = 0x00;
  BulkBuf[ 7] = 0x0A;           /* Additional Length */

  BulkBuf[ 8] = 0x00;
  BulkBuf[ 9] = 0x00;
  BulkBuf[10] = 0x00;
  BulkBuf[11] = 0x00;
  BulkBuf[12] = static_cast<uint8_t>(sense_data.asc);
  BulkBuf[13] = static_cast<uint8_t>(sense_data.ascq);
  BulkBuf[14] = 0x00;
  BulkBuf[15] = 0x00;
  BulkBuf[16] = 0x00;
  BulkBuf[17] = 0x00;


  BulkLen = 18;
  DataInTransfer();
}


/*
 *  MSC SCSI Inquiry Callback
 *  Parameters:      None (global variables)
 *  Return Value:    None
 */

void MSC_Inquiry (void) {

  if (!DataInFormat()) return;

  BulkBuf[ 0] = 0x00;          /* Direct Access Device */
  BulkBuf[ 1] = 0x80;          /* RMB = 1: Removable Medium */
  BulkBuf[ 2] = 0x00;          /* Version: No conformance claim to standard */
  BulkBuf[ 3] = 0x01;

  BulkBuf[ 4] = 36-4;          /* Additional Length */
  BulkBuf[ 5] = 0x80;          /* SCCS = 1: Storage Controller Component */
  BulkBuf[ 6] = 0x00;
  BulkBuf[ 7] = 0x00;

  BulkBuf[ 8] = 'M';           /* Vendor Identification */
  BulkBuf[ 9] = 'a';
  BulkBuf[10] = 'r';
  BulkBuf[11] = 'l';
  BulkBuf[12] = 'i';
  BulkBuf[13] = 'n';
  BulkBuf[14] = ' ';
  BulkBuf[15] = ' ';

  BulkBuf[16] = 'R';           /* Product Identification */
  BulkBuf[17] = 'e';
  BulkBuf[18] = '-';
  BulkBuf[19] = 'A';
  BulkBuf[20] = 'R';
  BulkBuf[21] = 'M';
  BulkBuf[22] = ' ';
  BulkBuf[23] = 'S';
  BulkBuf[24] = 'D';
  BulkBuf[25] = 'C';
  BulkBuf[26] = 'a';
  BulkBuf[27] = 'r';
  BulkBuf[28] = 'd';
  BulkBuf[29] = ' ';
  BulkBuf[30] = '0';
  BulkBuf[31] = '1';

  BulkBuf[32] = '1';           /* Product Revision Level */
  BulkBuf[33] = '.';
  BulkBuf[34] = '0';
  BulkBuf[35] = ' ';

  BulkLen = 36;
  DataInTransfer();
}


/*
 *  MSC SCSI Mode Sense (6-Byte) Callback
 *    Parameters:      None (global variables)
 *    Return Value:    None
 */

void MSC_ModeSense6 (void) {

  if (!DataInFormat()) return;

  BulkBuf[ 0] = 0x03;
  BulkBuf[ 1] = 0x00;
  BulkBuf[ 2] = 0x00;
  BulkBuf[ 3] = 0x00;
  BulkBuf[ 4] = 0x00;
  BulkBuf[ 5] = 0x00;
  BulkBuf[ 6] = 0x00;
  BulkBuf[ 7] = 0x00;

  BulkLen = 8;
  DataInTransfer();
}


/*
 *  MSC SCSI Mode Sense (10-Byte) Callback
 *    Parameters:      None (global variables)
 *    Return Value:    None
 */

void MSC_ModeSense10 (void) {

  if (!DataInFormat()) return;

  BulkBuf[ 0] = 0x00;
  BulkBuf[ 1] = 0x06;
  BulkBuf[ 2] = 0x00;
  BulkBuf[ 3] = 0x00;
  BulkBuf[ 4] = 0x00;
  BulkBuf[ 5] = 0x00;
  BulkBuf[ 6] = 0x00;
  BulkBuf[ 7] = 0x00;

  BulkLen = 8;
  DataInTransfer();
}


/*
 *  MSC SCSI Read Capacity Callback
 *    Parameters:      None (global variables)
 *    Return Value:    None
 */

void MSC_ReadCapacity (void) {
  if (!DataInFormat()) return;
  if (!MSC_CheckAvailable()) return;
  /* Last Logical Block */
  BulkBuf[ 0] = ((MSC_BlockCount - 1) >> 24) & 0xFF;
  BulkBuf[ 1] = ((MSC_BlockCount - 1) >> 16) & 0xFF;
  BulkBuf[ 2] = ((MSC_BlockCount - 1) >>  8) & 0xFF;
  BulkBuf[ 3] = ((MSC_BlockCount - 1) >>  0) & 0xFF;

  /* Block Length */
  BulkBuf[ 4] = (MSC_BLOCK_SIZE >> 24) & 0xFF;
  BulkBuf[ 5] = (MSC_BLOCK_SIZE >> 16) & 0xFF;
  BulkBuf[ 6] = (MSC_BLOCK_SIZE >>  8) & 0xFF;
  BulkBuf[ 7] = (MSC_BLOCK_SIZE >>  0) & 0xFF;

  BulkLen = 8;
  DataInTransfer();
}


/*
 *  MSC SCSI Read Format Capacity Callback
 *    Parameters:      None (global variables)
 *    Return Value:    None
 */

void MSC_ReadFormatCapacity (void) {
  uint32_t sz = MSC_BlockCount;
  if (!DataInFormat()) return;

  BulkBuf[ 0] = 0x00;
  BulkBuf[ 1] = 0x00;
  BulkBuf[ 2] = 0x00;
  BulkBuf[ 3] = 0x08;          /* Capacity List Length */

  if (MSC_BlockCount == 0) {
    // no SD card
    BulkBuf[ 8] = 0x03;       /* Descriptor Code: no media */
    sz = 0x7fffffff;          /* Max size */
  }
  else
    BulkBuf[ 8] = 0x02;       /* Descriptor Code: Formatted Media */
  /* Block Count */
  BulkBuf[ 4] = (sz >> 24) & 0xFF;
  BulkBuf[ 5] = (sz >> 16) & 0xFF;
  BulkBuf[ 6] = (sz >>  8) & 0xFF;
  BulkBuf[ 7] = (sz >>  0) & 0xFF;
  /* Block Length */
  BulkBuf[ 9] = (MSC_BLOCK_SIZE >> 16) & 0xFF;
  BulkBuf[10] = (MSC_BLOCK_SIZE >>  8) & 0xFF;
  BulkBuf[11] = (MSC_BLOCK_SIZE >>  0) & 0xFF;

  BulkLen = 12;
  DataInTransfer();
}


/*
 *  MSC Get Command Block Wrapper Callback
 *    Parameters:      None (global variables)
 *    Return Value:    None
 */

void MSC_GetCBW (void) {
  uint32_t n;

  for (n = 0; n < BulkLen; n++) {
    *((uint8_t *)&CBW + n) = BulkBuf[n];
  }
  if ((BulkLen == sizeof(CBW)) && (CBW.dSignature == MSC_CBW_Signature)) {
    /* Valid CBW */
    CSW.dTag = CBW.dTag;
    CSW.dDataResidue = CBW.dDataLength;
    if ((CBW.bLUN != 0) || (CBW.bCBLength < 1) || CBW.bCBLength > 16) {
fail: CSW.bStatus = CSW_CMD_FAILED;
      sense_data.set(Sense_KEY::ILLEGAL_REQUEST);
      MSC_SetCSW();
      _DBG("Unsupported SCSI OP code ");
      _DBH(CBW.CB[0]);
      _DBG("\n");
    } else {
      switch (CBW.CB[0]) {
        case SCSI_TEST_UNIT_READY:
          MSC_TestUnitReady();
          break;
        case SCSI_REQUEST_SENSE:
          MSC_RequestSense();
          break;
        case SCSI_FORMAT_UNIT:
          goto fail;
        case SCSI_INQUIRY:
          MSC_Inquiry();
          break;
        case SCSI_START_STOP_UNIT:
          MSC_StartStopUnit();
          break;
        case SCSI_MEDIA_REMOVAL:
          MSC_SD_Lock();
          break;
        case SCSI_MODE_SELECT6:
          goto fail;
        case SCSI_MODE_SENSE6:
          MSC_ModeSense6();
          break;
        case SCSI_MODE_SELECT10:
          goto fail;
        case SCSI_MODE_SENSE10:
          MSC_ModeSense10();
          break;
        case SCSI_READ_FORMAT_CAPACITIES:
          MSC_ReadFormatCapacity();
          break;
        case SCSI_READ_CAPACITY:
          MSC_ReadCapacity();
          break;
        case SCSI_READ10:
          if (MSC_RWSetup()) {
            if ((CBW.bmFlags & 0x80) != 0) {
              BulkStage = MSC_BS_DATA_IN;
              MSC_MemoryRead();
            } else {
              CSW.bStatus = CSW_PHASE_ERROR;
              MSC_SetCSW();
            }
          }
          break;
        case SCSI_WRITE10:
          if (MSC_RWSetup()) {
            if ((CBW.bmFlags & 0x80) == 0) {
              BulkStage = MSC_BS_DATA_OUT;
            } else {
              CSW.bStatus = CSW_PHASE_ERROR;
              MSC_SetCSW();
            }
          }
          break;
        case SCSI_VERIFY10:
          if (MSC_RWSetup()) {
            if ((CBW.bmFlags & 0x80) == 0) {
              BulkStage = MSC_BS_DATA_OUT;
              MemOK = TRUE;
            } else {
              CSW.bStatus = CSW_PHASE_ERROR;
              MSC_SetCSW();
            }
          }
          break;
        case 0x35: // SCSI_SYNCHRONIZECACHE10
          _DBG("SCSI_SYNCHRONIZECACHE10 Unsupported\n");
          CSW.bStatus = CSW_CMD_FAILED;
          sense_data.set(Sense_KEY::ILLEGAL_REQUEST);
          MSC_SetCSW();
          break;
        case 0x9E: // SCSI_SERVICEACTIONIN16
          _DBG("ServiceAction(16) Action: ");
          _DBH(CBW.CB[1]);
          _DBG(" Unsupported\n");
          CSW.bStatus = CSW_CMD_FAILED;
          sense_data.set(Sense_KEY::ILLEGAL_REQUEST);
          MSC_SetCSW();
          break;
        default:
          goto fail;
      }
    }
  } else {
    /* Invalid CBW */
    MSC_StallEP(MSC_EP_IN);
    MSC_StallEP(MSC_EP_OUT);
    BulkStage = MSC_BS_ERROR;
  }
}


/*
 *  MSC Set Command Status Wrapper Callback
 *    Parameters:      None (global variables)
 *    Return Value:    None
 */

void MSC_SetCSW (void) {
  uint32_t sep;
  BulkStage = MSC_BS_CSW;
  CSW.dSignature = MSC_CSW_Signature;
  // Stall endpoint if required see: http://www.usb.org/developers/docs/devclass_docs/usbmassbulk_10.pdf
  if (CSW.dDataResidue != 0) {
    if ((CBW.bmFlags & 0x80) == 0) {
      MSC_StallEP(MSC_EP_OUT);
    }
    else {
      MSC_StallEP(MSC_EP_IN);
    }
  }
  if (CSW.bStatus == CSW_PHASE_ERROR) {
    MSC_StallEP(MSC_EP_IN);
  }

  USB_WriteEP(MSC_EP_IN, (uint8_t *)&CSW, sizeof(CSW));
}


/*
 *  MSC Bulk In Callback
 *    Parameters:      None (global variables)
 *    Return Value:    None
 */

void MSC_BulkIn (void) {
  switch (BulkStage) {
    case MSC_BS_DATA_IN:
      switch (CBW.CB[0]) {
        case SCSI_READ10:
          MSC_MemoryRead();
          break;
      }
      break;
    case MSC_BS_DATA_IN_LAST:
      MSC_SetCSW();
      break;
    case MSC_BS_DATA_IN_LAST_STALL:
      MSC_StallEP(MSC_EP_IN);
      MSC_SetCSW();
      break;
    case MSC_BS_CSW:
      BulkStage = MSC_BS_CBW;
      break;
  }
}


/*
 *  MSC Bulk Out Callback
 *    Parameters:      None (global variables)
 *    Return Value:    None
 */

void MSC_BulkOut (void) {
  switch (BulkStage) {
    case MSC_BS_CSW:
      // This phase will normally coincide with a BulkIn request. However
      // if interrupts are delayed then the the order of BulkIn/BulkOut
      // requests is not guaranteed. So we also handle it here.
      BulkStage = MSC_BS_CBW;
      /* FALLTHROUGH */
    case MSC_BS_CBW:
      BulkLen = (uint8_t)USB_ReadEP(MSC_EP_OUT, BulkBuf);
      MSC_GetCBW();
      break;
    case MSC_BS_DATA_OUT:
      switch (CBW.CB[0]) {
        case SCSI_WRITE10:
          MSC_MemoryWrite();
          break;
        case SCSI_VERIFY10:
          MSC_MemoryVerify();
          break;
        default:
        _DBG("unexpected op "); _DBD32(CBW.CB[0]); _DBG("\n");
      }
      break;
    default:
      _DBG("BO Phase error ");_DBD32(BulkStage); _DBG("\n");
      BulkLen = (uint8_t)USB_ReadEP(MSC_EP_OUT, BulkBuf);
      _DBG("Data len "); _DBD32(BulkLen); _DBG("\n");
      MSC_StallEP(MSC_EP_OUT);
      CSW.bStatus = CSW_PHASE_ERROR;
      MSC_SetCSW();
      break;
  }
}
