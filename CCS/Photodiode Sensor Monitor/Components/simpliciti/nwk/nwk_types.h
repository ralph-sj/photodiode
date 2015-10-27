/**************************************************************************************************
  Filename:       nwk_types.h
  Revised:        $Date: 2008-07-09 17:24:12 -0700 (Wed, 09 Jul 2008) $
  Revision:       $Revision: 17502 $
  Author:         $Author: lfriedman $

  Description:    This header file defines the SimpliciTI typedefs.

  Copyright 2004-2007 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights granted under
  the terms of a software license agreement between the user who downloaded the software,
  his/her employer (which must be your employer) and Texas Instruments Incorporated (the
  "License"). You may not use this Software unless you agree to abide by the terms of the
  License. The License limits your use, and you acknowledge, that the Software may not be
  modified, copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio frequency
  transceiver, which is integrated into your product. Other than for the foregoing purpose,
  you may not use, reproduce, copy, prepare derivative works of, modify, distribute,
  perform, display or sell this Software and/or its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS”
  WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY
  WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
  IN NO EVENT SHALL TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE
  THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY
  INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST
  DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY
  THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

#ifndef NWK_TYPES_H
#define NWK_TYPES_H

#define NWK_TX_RETRY_COUNT    10
#define NWK_RX_RETRY_COUNT    10

#define NWK_APP_REPLY_BIT  (0x80)

#define NET_ADDR_SIZE      MRFI_ADDR_SIZE   /* size of address in bytes */

#ifdef  FREQUENCY_AGILITY
#define NWK_FREQ_TBL_SIZE  MRFI_NUM_LOGICAL_CHANS
#else
#define NWK_FREQ_TBL_SIZE  1
#endif

typedef struct
{
  uint8_t  addr[NET_ADDR_SIZE];
} addr_t;

typedef uint8_t linkID_t;
typedef uint8_t ccRadioStatus_t;

enum ioctlObject  {
  IOCTL_OBJ_FREQ,
  IOCTL_OBJ_CRYPTKEY,
  IOCTL_OBJ_RAW_IO,
  IOCTL_OBJ_RADIO,
  IOCTL_OBJ_AP_JOIN,
  IOCTL_OBJ_ADDR,
  IOCTL_OBJ_CONNOBJ,
  IOCTL_OBJ_FWVER,
  IOCTL_OBJ_PROTOVER
};

enum ioctlAction  {
  IOCTL_ACT_SET,
  IOCTL_ACT_GET,
  IOCTL_ACT_READ,
  IOCTL_ACT_WRITE,
  IOCTL_ACT_RADIO_SLEEP,
  IOCTL_ACT_RADIO_AWAKE,
  IOCTL_ACT_RADIO_SIGINFO,
  IOCTL_ACT_RADIO_RSSI,
  IOCTL_ACT_RADIO_RXON,
  IOCTL_ACT_RADIO_RXIDLE,
  IOCTL_ACT_ON,
  IOCTL_ACT_OFF,
  IOCTL_ACT_SCAN,
  IOCTL_ACT_DELETE
};

typedef enum ioctlObject   ioctlObject_t;
typedef enum ioctlAction   ioctlAction_t;

typedef struct
{
  addr_t   *addr;
  uint8_t  *msg;
  uint8_t   len;
  uint8_t   port;
} ioctlRawSend_t;

typedef struct
{
  addr_t  *addr;
  uint8_t *msg;
  uint8_t  len;
  uint8_t  port;
  uint8_t  hopCount;
} ioctlRawReceive_t;

/*
 * Signal information support
 */
typedef int8_t rssi_t;

typedef struct
{
  rssi_t  rssi;
  uint8_t lqi;
} rxMetrics_t;

typedef struct
{
  linkID_t     lid;   /* input: port for which signal info desired */
  rxMetrics_t  sigInfo;
} ioctlRadioSiginfo_t;

/*
 * Frequency Agility support
 */
typedef struct
{
  uint8_t logicalChan;
} freqEntry_t;

typedef struct
{
  uint8_t      numChan;
  freqEntry_t *freq;
} ioctlScanChan_t;

enum smplStatus  {
  SMPL_SUCCESS,
  SMPL_TIMEOUT,
  SMPL_BAD_PARAM,
  SMPL_NOMEM,
  SMPL_NO_FRAME,
  SMPL_NO_LINK,
  SMPL_NO_JOIN,
  SMPL_NO_CHANNEL,
  SMPL_NO_PEER_UNLINK,
  SMPL_TX_CCA_FAIL,
  SMPL_NO_PAYLOAD,
  SMPL_NO_AP_ADDRESS
};

typedef enum smplStatus    smplStatus_t;

/* NWK application frame handling status codes */
enum fhStatus
{
  FHS_RELEASE,   /* handled in interrupt thread */
  FHS_KEEP,      /* handled by background application */
  FHS_REPLAY     /* non-ED case: NWK frame not for me that should be replayed */
};

typedef enum fhStatus   fhStatus_t;

/********    BEGIN: Object support for parameter context in queue management *********/
enum rcvType
{
  RCV_NWK_PORT,
  RCV_APP_LID,
  RCV_RAW_POLL_FRAME
};

typedef enum rcvType rcvType_t;

typedef struct
{
  rcvType_t   type;
  union
  {
    linkID_t      lid;
    uint8_t       port;
    mrfiPacket_t *pkt;
  } t;
} rcvContext_t;
/********    END: Object support for parameter context in queue management *********/

#define SMPL_FWVERSION_SIZE  4

typedef struct
{
  uint8_t  protocolVersion;
  uint8_t  fwVerString[SMPL_FWVERSION_SIZE];
} smplVersionInfo_t;

/****************************************************************************************
 *                           SOME USEFUL MACROS
 ****************************************************************************************/

/* Delay loop support. Requires mrfi.h */
#define NWK_DELAY(spin)   MRFI_DelayMs(spin)
#define NWK_REPLY_DELAY() MRFI_ReplyDelay();

/* Network applications may need to remember radio state because the user
 * application may choose to turn Rx off. These macros help get and restore
 * the radio Rx state. The macros should be in the same code block at the same level.
 * The argument 's' is the 'current' radio state and should be set in the code block
 * with a call to MRFI_GetRadioState() _before_ using the macros.
 *
 * Used extensively by NWK but user applications may use them as well. But it is
 * much more liekly that an application will know the radio state since it likely
 * will have set it with IOCTL calls. Requires mrfi.h.
 */
#define NWK_CHECK_FOR_SETRX(s)  if (MRFI_RADIO_STATE_RX != s)    \
                                {                                \
                                  if (MRFI_RADIO_STATE_OFF == s) \
                                  {                              \
                                    MRFI_WakeUp();               \
                                  }                              \
                                  MRFI_RxOn();                   \
                                }

#define NWK_CHECK_FOR_RESTORE_STATE(s) if (MRFI_RADIO_STATE_RX != s)    \
                                       {                                \
                                         if (MRFI_RADIO_STATE_OFF == s) \
                                         {                              \
                                           MRFI_Sleep();                \
                                         }                              \
                                         else                           \
                                         {                              \
                                           MRFI_RxIdle();               \
                                         }                              \
                                       }
#endif
