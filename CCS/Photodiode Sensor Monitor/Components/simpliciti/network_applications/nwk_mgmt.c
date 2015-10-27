/**************************************************************************************************
  Filename:       nwk_mgmt.c
  Revised:        $Date: 2008-08-01 10:56:17 -0700 (Fri, 01 Aug 2008) $
  Revision:       $Revision: 17682 $
  Author:         $Author: lfriedman $

  Description:    This file supports the SimpliciTI Mgmt network application.

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


/******************************************************************************
 * INCLUDES
 */
#include <string.h>
#include "bsp.h"
#include "mrfi.h"
#include "nwk_types.h"
#include "nwk_api.h"
#include "nwk_frame.h"
#include "nwk.h"
#include "nwk_mgmt.h"
#include "nwk_join.h"
#include "nwk_globals.h"
#include "nwk_QMgmt.h"


/******************************************************************************
 * MACROS
 */

/******************************************************************************
 * CONSTANTS AND DEFINES
 */

/******************************************************************************
 * TYPEDEFS
 */

/******************************************************************************
 * LOCAL VARIABLES
 */
#ifndef ACCESS_POINT
static addr_t const *sAPAddr;
#endif

static volatile uint8_t sTid = 0;

/******************************************************************************
 * LOCAL FUNCTIONS
 */
static void  smpl_send_mgmt_reply(mrfiPacket_t *);
#ifdef ACCESS_POINT
static void  send_poll_reply(mrfiPacket_t *);
#endif

/******************************************************************************
 * GLOBAL VARIABLES
 */

/******************************************************************************
 * GLOBAL FUNCTIONS
 */

/******************************************************************************
 * @fn          nwk_mgmtInit
 *
 * @brief       Initialize Management functions.
 *
 * input parameters
 *
 * output parameters
 *
 * @return   void
 */

void nwk_mgmtInit(void)
{
  sTid = MRFI_RandomByte();

  return;
}

/******************************************************************************
 * @fn          nwk_processMgmt
 *
 * @brief       Process Management frame. Just save the frame for the Management
 *              app it it is a reply. If it isn't a reply, send the reply in this
 *              thread.
 *
 * input parameters
 * @param   frame   - pointer to frame to be processed
 *
 * output parameters
 *
 * @return   Non-zero if frame should be kept otherwise 0 to delete frame.
 */
fhStatus_t nwk_processMgmt(mrfiPacket_t *frame)
{
  fhStatus_t   rc;
  uint8_t      replyType;

  /* If we sent this then this is the reply. Validate the
   * packet for reception by client app. If we didn't send
   * it then we are the target. send the reply.
   */
  if (SMPL_MY_REPLY == (replyType=nwk_isValidReply(frame, sTid, MB_APP_INFO_OS, MB_TID_OS)))
  {
    /* It's a match and it's a reply. Validate the received packet by
     * returning a 1 so it can be received by the client app.
     */
    MRFI_PostKillSem();
    rc = FHS_KEEP;
  }
#if !defined( END_DEVICE )
  else if (SMPL_A_REPLY == replyType)
  {
    /* no match. if i'm not an ED this is a reply that should be passed on. */
    rc = FHS_REPLAY;
  }
#endif  /* !END_DEVICE */
  else
  {
    /* no, we didn't send it. send reply if it's intended for us */
    if (!memcmp(MRFI_P_DST_ADDR(frame), nwk_getMyAddress(), NET_ADDR_SIZE))
    {
      smpl_send_mgmt_reply(frame);

      /* we're done with the frame. */
      rc = FHS_RELEASE;
    }
    else
    {
      rc = FHS_REPLAY;
    }
  }

  (void) replyType;  /* keep compiler happy */

  return rc;
}

static void smpl_send_mgmt_reply(mrfiPacket_t *frame)
{
#ifdef ACCESS_POINT
  /* what kind of management frame is this? */
  switch (*(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+MB_APP_INFO_OS))
  {
    case MGMT_REQ_POLL:
      send_poll_reply(frame);
      break;

    default:
      break;
  }
#endif  /* ACCESS_POINT */
  return;
}

#ifdef ACCESS_POINT
static void send_poll_reply(mrfiPacket_t *frame)
{
  frameInfo_t *pOutFrame;

  /* make sure this guy is really a client. we can tell from the source address. */
  if (!nwk_isSandFClient(MRFI_P_SRC_ADDR(frame)))
  {
    /* TODO: maybe send an error frame? */
    return;
  }

  if (pOutFrame = nwk_getSandFFrame(frame, M_POLL_PORT_OS))
  {
    /* We need to adjust the order in the queue in this case. Currently
     * we know it is in the input queue and that this adjustment is safe
     * because we're in an ISR thread. This is a fragile fix, though, and
     * should be revisited when time permits.
     */
    nwk_QadjustOrder(INQ, pOutFrame->orderStamp);

    /* reset hop count... */
    PUT_INTO_FRAME(MRFI_P_PAYLOAD(&pOutFrame->mrfiPkt), F_HOP_COUNT, MAX_HOPS_FROM_AP);
    nwk_sendFrame(pOutFrame, MRFI_TX_TYPE_FORCED);
  }
  else
  {
    nwk_SendEmptyPollRspFrame(frame);
  }

  return;
}

#else  /* ACCESS_POINT */

smplStatus_t nwk_poll(uint8_t port, uint8_t *addr)
{
  uint8_t        msg[MGMT_POLL_FRAME_SIZE];
  ioctlRawSend_t send;

  msg[MB_APP_INFO_OS] = MGMT_REQ_POLL;
  msg[MB_TID_OS]      = sTid;
  msg[M_POLL_PORT_OS] = port;
  memcpy(msg+M_POLL_ADDR_OS, addr, NET_ADDR_SIZE);

  if (!sAPAddr)
  {
    sAPAddr = nwk_getAPAddress();
    if (!sAPAddr)
    {
      return SMPL_NO_AP_ADDRESS;
    }
  }
  send.addr = (addr_t *)sAPAddr;
  send.msg  = msg;
  send.len  = sizeof(msg);
  send.port = SMPL_PORT_MGMT;

  return SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE, &send);
}

#endif /* ACCESS_POINT */
