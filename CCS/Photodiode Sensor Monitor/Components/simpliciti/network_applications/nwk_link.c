/**************************************************************************************************
  Filename:       nwk_link.c
  Revised:        $Date: 2008-08-04 11:02:11 -0700 (Mon, 04 Aug 2008) $
  Revision:       $Revision: 17701 $
  Author:         $Author: lfriedman $

  Description:    This file supports the SimpliciTI Link network application.

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
#include "nwk_link.h"
#include "nwk_globals.h"
#include "nwk_join.h"     /* nwk_purgeJoined()  */

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
static uint32_t          sLinkToken = 0;
static volatile uint8_t  sListenActive = 0;
static volatile linkID_t sServiceLinkID = 0;
static volatile uint8_t  sTid = 0;

/******************************************************************************
 * LOCAL FUNCTIONS
 */

#define  SENT_REPLY       1
#define  SENT_NO_REPLY    2
static uint8_t    smpl_send_link_reply(mrfiPacket_t *);
static void       smpl_send_unlink_reply(mrfiPacket_t *);
static fhStatus_t handleLinkRequest(mrfiPacket_t *);

/******************************************************************************
 * GLOBAL VARIABLES
 */

/******************************************************************************
 * GLOBAL FUNCTIONS
 */


/******************************************************************************
 * @fn          nwk_linkInit
 *
 * @brief       Initialize link app. Set link token to the default.
 *
 * input parameters
 *
 * output parameters
 *
 * @return   void
 */
void nwk_linkInit(void)
{
  if (!sLinkToken)
  {
    sLinkToken = DEFAULT_LINK_TOKEN;
  }

  /* set a non-zero TID. */
  while (!(sTid = MRFI_RandomByte()))  ;

  return;
}


/******************************************************************************
 * @fn          nwk_setLinkToken
 *
 * @brief       Sets the link token received in a Join reply.
 *
 * input parameters
 * @param   token   - Link token to be used on this network to link to any peer.
 *
 * output parameters
 *         no room in output queue.
 *
 * @return   void
 */
void nwk_setLinkToken(uint32_t token)
{
  /* only set if the supplied token is non-zero. */
  if (token)
  {
    sLinkToken = token;
  }

  return;
}


/******************************************************************************
 * @fn          nwk_link
 *
 * @brief       Called from the application level to accomplish the link
 *
 * input parameters
 *
 * output parameters
 * @param   lid     - pointer to Link ID (port) assigned for this link
 *
 * @return   Status of the operation.
 */
smplStatus_t nwk_link(linkID_t *lid)
{
  uint8_t       msg[LINK_FRAME_SIZE];
  connInfo_t   *pCInfo = nwk_getNextConnection();
  smplStatus_t  rc;

  if (pCInfo)
  {
    addr_t              addr;
    union
    {
      ioctlRawSend_t    send;
      ioctlRawReceive_t recv;
    } ioctl_info;

    if (!nwk_allocateLocalRxPort(LINK_SEND, pCInfo))
    {
      nwk_freeConnection(pCInfo);
      return SMPL_NOMEM;
    }

    memcpy(addr.addr, nwk_getBCastAddress(), NET_ADDR_SIZE);
    ioctl_info.send.addr = &addr;
    ioctl_info.send.msg  = msg;
    ioctl_info.send.len  = sizeof(msg);
    ioctl_info.send.port = SMPL_PORT_LINK;

    /* copy link token in */
    memcpy(msg+L_LINK_TOKEN_OS, &sLinkToken, sizeof(sLinkToken));

    /* set port to which the remote device should send */
    msg[L_RMT_PORT_OS] = pCInfo->portRx;

    /* set the transaction ID. this allows target to figure out duplicates */
    msg[LB_TID_OS] = sTid;

    /* set my Rx type */
    msg[L_MY_RXTYPE_OS] = nwk_getMyRxType();

    /* set request byte */
    msg[LB_REQ_OS] = LINK_REQ_LINK;

    /* protocol version number */
    msg[L_PROTOCOL_VERSION_OS] = nwk_getProtocolVersion();

    if (SMPL_SUCCESS != (rc=SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE, &ioctl_info.send)))
    {
      return rc;
    }

    {
      uint8_t radioState = MRFI_GetRadioState();

      ioctl_info.recv.port = SMPL_PORT_LINK;
      ioctl_info.recv.msg  = msg;
      ioctl_info.recv.addr = (addr_t *)pCInfo->peerAddr;

      NWK_CHECK_FOR_SETRX(radioState);
      NWK_REPLY_DELAY();
      NWK_CHECK_FOR_RESTORE_STATE(radioState);

      if (SMPL_SUCCESS == SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_READ, &ioctl_info.recv))
      {
        uint8_t firstByte = msg[LB_REQ_OS] & (~NWK_APP_REPLY_BIT);

        /* Sanity check for correct reply frame. Older version
         * has the length instead of the request as the first byte.
         */
        if ((firstByte != LINK_REQ_LINK) &&
            (firstByte != LINK_REPLY_LEGACY_MSG_LENGTH)
           )
        {
          /* invalidate connection object */
          nwk_freeConnection(pCInfo);
          return SMPL_NO_LINK;

        }
      }
      else
      {
        /* no successful receive */
        nwk_freeConnection(pCInfo);
        return SMPL_TIMEOUT;
      }

      pCInfo->isValid = 1;
      pCInfo->portTx  = msg[LR_RMT_PORT_OS];    /* link reply returns remote port */
      *lid            = pCInfo->thisLinkID;     /* return our local port number */

      /* Set hop count. If it's a polling device set the count to the
       * distance to the AP. Otherwise, set it to the max less the remaining
       * which will be the path taken for this frame. It will be no worse
       * then tha max and probably will be better.
       */
      if (F_RX_TYPE_POLLS == msg[LR_MY_RXTYPE_OS])
      {
        pCInfo->hops2target = MAX_HOPS;
      }
      else
      {
        /* Can't really use this trick because the device could move. If the
         * devices are all static this may work unless the initial reception
         * was marginal.
         */
#ifdef DEVICE_DOES_NOT_MOVE
        pCInfo->hops2target = MAX_HOPS - ioctl_info.recv.hopCount;
#else
        pCInfo->hops2target = MAX_HOPS;
#endif
      }
    }

    /* guard against duplicates... */
    ++sTid;
    if (!sTid)
    {
      sTid = 1;
    }
    return SMPL_SUCCESS;
  }

  return SMPL_NOMEM;
}

/******************************************************************************
 * @fn          smpl_send_unlink_reply
 *
 * @brief       Send the unlink reply to the device trying to unlink
 *
 * input parameters
 * @param   frame   - frame received from linker
 *
 * output parameters
 *
 * @return   void
 */
static void smpl_send_unlink_reply(mrfiPacket_t *frame)
{
  connInfo_t  *pCInfo;
  frameInfo_t *pOutFrame;
  uint8_t      msg[UNLINK_REPLY_FRAME_SIZE];
  smplStatus_t rc = SMPL_NO_PEER_UNLINK;

  /* match the remote port and source address with a connection table entry */
  if (pCInfo = nwk_findPeer((addr_t *)MRFI_P_SRC_ADDR(frame), *(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+UL_RMT_PORT_OS)))
  {
    rc = SMPL_SUCCESS;
  }

  /* set reply bit */
  msg[LB_REQ_OS] = LINK_REQ_UNLINK | NWK_APP_REPLY_BIT;

  /* sender's TID */
  msg[LB_TID_OS] = *(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+LB_TID_OS);

  /* result of freeing local connection */
  msg[ULR_RESULT_OS] = rc;

  if (pOutFrame = nwk_buildFrame(SMPL_PORT_LINK, msg, sizeof(msg), MAX_HOPS))
  {
    /* destination address is the source adddress of the received frame. */
    memcpy(MRFI_P_DST_ADDR(&pOutFrame->mrfiPkt), MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE);

    nwk_sendFrame(pOutFrame, MRFI_TX_TYPE_FORCED);

    /* release the connection structure */
#ifdef ACCESS_POINT
    nwk_purgeJoined(pCInfo->peerAddr);
#endif
    nwk_freeConnection(pCInfo);
  }
}

/******************************************************************************
 * @fn          smpl_send_link_reply
 *
 * @brief       Send the link reply to the device trying to link. This routine
 *              will handle duplicates.
 *
 * input parameters
 * @param   frame   - frame received from linker
 *
 * output parameters
 *
 * @return   Returns SENT_REPLY if reply sent, else SENT_NO_REPLY.
 *           The return value is used as this routine unwinds to know
 *           whether to replay the frame. An RE or AP can host an ED
 *           object in which case it might send a reply (possibly from
 *           a duplicate frame). If we do reply we do not want to replay.
 */
static uint8_t smpl_send_link_reply(mrfiPacket_t *frame)
{
  frameInfo_t *pOutFrame;
  connInfo_t  *pCInfo;
  uint8_t      remotePort;
  uint8_t      msg[LINK_REPLY_FRAME_SIZE];

  /* Is this a legacy frame? If so continue. Otherwise check version.*/
  if ((MRFI_GET_PAYLOAD_LEN(frame) - F_APP_PAYLOAD_OS) > LINK_LEGACY_MSG_LENGTH)
  {
    /* see if protocol version is correct... */
    if (*(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+L_PROTOCOL_VERSION_OS) != nwk_getProtocolVersion())
    {
      /* Accommodation of protocol version differences can be noted or accomplished here.
       * This field was also checked in the join transaction but it is checked again here
       * because that check may not have occurred if thre is no AP in this topology.
       * Otherwise, no match and the board goes back
       */
      return SENT_NO_REPLY;
    }
  }

  /* see if token is correct */
  if (memcmp(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+L_LINK_TOKEN_OS, &sLinkToken, sizeof(sLinkToken)))
  {
    /* nope. not Tx-only. match failed. we're done. */
    /*TODO: maybe return a NAK error packet?? */
    return SENT_NO_REPLY;
  }

  /* if we get here the token matched. */

  /* is this a duplicate request? */
  remotePort = *(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+L_RMT_PORT_OS);
  if (pCInfo=nwk_isLinkDuplicate(MRFI_P_SRC_ADDR(frame), remotePort))
  {
    /* resend reply */
    msg[LB_REQ_OS] = LINK_REQ_LINK | NWK_APP_REPLY_BIT;

    /* sender's TID */
    msg[LB_TID_OS] = *(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+LB_TID_OS);

    /* Send reply with the local port number so the remote device knows where to
     * send packets.
     */
    msg[LR_RMT_PORT_OS] = pCInfo->portRx;

    /* put my Rx type in there. used to know how to set hops when sending back. */
    msg[LR_MY_RXTYPE_OS] = nwk_getMyRxType();

    if (pOutFrame = nwk_buildFrame(SMPL_PORT_LINK, msg, sizeof(msg), MAX_HOPS-(GET_FROM_FRAME(MRFI_P_PAYLOAD(frame),F_HOP_COUNT))))
    {
      /* destination address is the source adddress of the received frame. */
      memcpy(MRFI_P_DST_ADDR(&pOutFrame->mrfiPkt), MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE);
      nwk_sendFrame(pOutFrame, MRFI_TX_TYPE_FORCED);
    }
    return SENT_REPLY;
  }

  if (!sListenActive)
  {
    /* We've checked for duplicate and resent reply. In that case we weren't listening
     * so just go back`.
     */
    return SENT_NO_REPLY;
  }

  /* room to link? */
  pCInfo = nwk_getNextConnection();
  if (pCInfo)
  {
    /* yes there's room and it's not a dup. address. */
    memcpy(&pCInfo->peerAddr, MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE);

    if (!nwk_allocateLocalRxPort(LINK_REPLY, pCInfo))
    {
      nwk_freeConnection(pCInfo);
      return SENT_NO_REPLY;
    }

    /* The local Rx port is the one returned in the connection structure. The
     * caller is waiting on this to be set. The code here is running in an ISR
     * thread so the caller will see this change after RETI.
     */
    sServiceLinkID = pCInfo->thisLinkID;

    /* save the remote Tx port */
    pCInfo->portTx = remotePort;

    /* connection is valid... */
    pCInfo->isValid = 1;

    /* Set hop count. If it's a polling device set the count to the
     * distance to the AP. otherwise, set it to the max less the remaining
     * which will be the path taken for this frame. It will be no worse
     * then tha max and probably will be better.
     */
    if (F_RX_TYPE_POLLS == *(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+L_MY_RXTYPE_OS))
    {
      /* It polls. so. we'll be sending to the AP which will store the
       * frame. The AP is only MAX_HOPS_FROM_AP hops away from us.
       */
      pCInfo->hops2target = MAX_HOPS_FROM_AP;
    }
    else
    {
      /* Can't really use this trick because the device could move. If the
       * devices are all static this may work unless the initial reception
       * was marginal.
       */
#ifdef DEVICE_DOES_NOT_MOVE
      pCInfo->hops2target = MAX_HOPS - GET_FROM_FRAME(MRFI_P_PAYLOAD(frame), F_HOP_COUNT);
#else
      pCInfo->hops2target = MAX_HOPS;
#endif
    }
    /* Send reply with the local port number so the remote device knows where to
     * send packets.
     */
    msg[LR_RMT_PORT_OS]  = pCInfo->portRx;

    /* put my Rx type in there. used to know how to set hops when sending back. */
    msg[LR_MY_RXTYPE_OS] = nwk_getMyRxType();

    msg[LB_REQ_OS] = LINK_REQ_LINK | NWK_APP_REPLY_BIT;

    /* sender's TID */
    msg[LB_TID_OS]       = *(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+LB_TID_OS);

    if (pOutFrame = nwk_buildFrame(SMPL_PORT_LINK, msg, sizeof(msg), MAX_HOPS-(GET_FROM_FRAME(MRFI_P_PAYLOAD(frame),F_HOP_COUNT))))
    {
      /* destination address is the source adddress of the received frame. */
      memcpy(MRFI_P_DST_ADDR(&pOutFrame->mrfiPkt), MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE);
      if (SMPL_SUCCESS != nwk_sendFrame(pOutFrame, MRFI_TX_TYPE_FORCED))
      {
        /* better release the connection structure */
        nwk_freeConnection(pCInfo);
      }
    }
    else
    {
      /* better release the connection structure */
      nwk_freeConnection(pCInfo);
    }
  }
  /* we're done with the packet */
  return SENT_REPLY;
}

/******************************************************************************
 * @fn          nwk_processLink
 *
 * @brief       Process Link frame. Just save the frame for the Link app if it
 *              a reply. If it isn't a reply, send the reply in this thread.
 *
 * input parameters
 * @param   frame   - pointer to frame to be processed
 *
 * output parameters
 *
 * @return   Non-zero if frame should be kept otherwise 0 to delete frame.
 */
fhStatus_t nwk_processLink(mrfiPacket_t *frame)
{
  fhStatus_t   rc;
  uint8_t      replyType;

  /* If we sent this then this is the reply. Validate the
   * packet for reception by client app. If we didn't send
   * it then we are the target. send the reply.
   */
  if (SMPL_MY_REPLY == (replyType=nwk_isValidReply(frame, sTid, LB_REQ_OS, LB_TID_OS)))
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
    /* No, we didn't send it. Process request assuming it's
     * intended for us.
     */
    rc = handleLinkRequest(frame);
  }

  (void) replyType;  /* keep compiler happy when ED built... */

  return rc;
}


/******************************************************************************
 * @fn          nwk_getLocalLinkID
 *
 * @brief       This routine checks to see if a service port has been assigned
 *              as a result of a link reply frame being received. It is the means
 *              by which the user thread knows that the waiting is over for the
 *              link listen. the value is set in an interrupt thread.
 *
 * input parameters
 *
 * output parameters
 *
 * @return   Local port assigned when the link reply was received.
 */
linkID_t nwk_getLocalLinkID(void)
{
  linkID_t lid = 0;

  if (sServiceLinkID)
  {
    lid            = sServiceLinkID;
    sServiceLinkID = 0;
    nwk_setListenContext(LINK_LISTEN_OFF);
  }

  return lid;
}


/******************************************************************************
 * @fn          nwk_setListenContext
 *
 * @brief       Sets the context when a LinkListen is executed. This prevents
 *              processing other link frames from being confused with the real
 *              one. Without this semaphore other broadcast link messages
 *              could wait int the input queue and accidently be processed if
 *              a listen is done later.
 *
 * input parameters
 *
 * @param   context - listen on or off
 *
 * output parameters
 *
 * @return   void
 */
void nwk_setListenContext(uint8_t context)
{
  sListenActive = (context == LINK_LISTEN_ON) ? 1 : 0;
}

/******************************************************************************
 * @fn          handleLinkRequest
 *
 * @brief       Dispatches handler for specfic link request
 *
 * input parameters
 *
 * @param   frame - Link frame received
 *
 * output parameters
 *
 * @return   void
 */
static fhStatus_t handleLinkRequest(mrfiPacket_t *frame)
{
  fhStatus_t rc = FHS_RELEASE;
  uint8_t    isReplySent;

  if (LINK_LEGACY_MSG_LENGTH == (MRFI_GET_PAYLOAD_LEN(frame) - F_APP_PAYLOAD_OS))
  {
    /* Legacy frame. Spoof a link request */
    *(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS) = LINK_REQ_LINK;
  }

  switch (*(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS))
  {
    case LINK_REQ_LINK:
      isReplySent = smpl_send_link_reply(frame);
#ifndef END_DEVICE
      /* If I am an AP or RE and not listening I need to replay frame.
       * The exception is if I am an AP or RE hosting an End Device
       * object and I just sent a reply frame to a duplicate link frame
       * for which I was not listening. In this case don't replay.
       */
      if (!sListenActive && (SENT_REPLY != isReplySent))
      {
        rc = FHS_REPLAY;
      }
#endif   /* !END_DEVICE */
      break;

    case LINK_REQ_UNLINK:
      smpl_send_unlink_reply(frame);
      break;

    default:
      break;
  }

  /* keep compiler happy if I'm compiled as an End Device */
  (void) isReplySent;

  return rc;
}
