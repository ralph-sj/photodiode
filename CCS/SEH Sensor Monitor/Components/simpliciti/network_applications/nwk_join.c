
/**************************************************************************************************
  Filename:       nwk_join.c
  Revised:        $Date: 2008-08-04 11:02:11 -0700 (Mon, 04 Aug 2008) $
  Revision:       $Revision: 17701 $
  Author:         $Author: lfriedman $

  Description:    This file supports the SimpliciTI Join network application.

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
#include "nwk_join.h"
#include "nwk_globals.h"
#include "nwk_freq.h"

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

static          uint32_t sJoinToken = 0;
static          uint8_t (*spCallback)(linkID_t) = 0;
static volatile uint8_t  sTid = 0;

#ifdef ACCESS_POINT
static uint32_t sLinkToken = 0;
static addr_t   sSandFClients[NUM_STORE_AND_FWD_CLIENTS] = {0};
static uint8_t  sCurNumSandFClients = 0;
static uint8_t  sJoinOK = 0;
#ifdef AP_IS_DATA_HUB
static addr_t   sJoinedED[NUM_CONNECTIONS] = {0};
static uint8_t  sNumJoined = 0;
#endif
#endif

/******************************************************************************
 * LOCAL FUNCTIONS
 */
#ifdef ACCESS_POINT
static void smpl_send_join_reply(mrfiPacket_t *frame);
static void generateLinkToken(void);
static void handleJoinRequest(mrfiPacket_t *);
#ifdef AP_IS_DATA_HUB
static void    saveAddress(mrfiPacket_t *);
static uint8_t isJoined(mrfiPacket_t *);
#endif
#else  /*  ACCESS_POINT */

#endif  /*  ACCESS_POINT */

/******************************************************************************
 * GLOBAL VARIABLES
 */

/******************************************************************************
 * GLOBAL FUNCTIONS
 */

/******************************************************************************
 * @fn          nwk_joinInit
 *
 * @brief       Initialize Join application.
 *
 * input parameters
 *
 * output parameters
 *
 * @return   void
 */
void nwk_joinInit(uint8_t (*pf)(linkID_t))
{
  sJoinToken = DEFAULT_JOIN_TOKEN;
  spCallback = pf;
  (void) spCallback;  /* keep compiler happy if we don't use this */

  sTid = MRFI_RandomByte() ;

#ifdef ACCESS_POINT
  generateLinkToken();
  nwk_setLinkToken(sLinkToken);
  sJoinOK = 1;
#endif

  return;
}

/******************************************************************************
 * @fn          generateLinkToken
 *
 * @brief       Generate the link token to be used for the network controlled
 *              by this Access Point.
 *
 * input parameters
 *
 * output parameters
 *
 * @return   void
 */
#ifdef ACCESS_POINT
static void generateLinkToken(void)
{
  sLinkToken = 0xDEADBEEF;

  return;
}

/******************************************************************************
 * @fn          smpl_send_join_reply
 *
 * @brief       Send the Join reply. Include the Link token. If the device is
 *              a polling sleeper put it into the list of store-and-forward
 *              clients.
 *
 * input parameters
 * @param   frame     - join frame for which a reply is needed...maybe
 *
 * output parameters
 *
 * @return   void
 */
static void smpl_send_join_reply(mrfiPacket_t *frame)
{
  frameInfo_t *pOutFrame;
  uint8_t      msg[JOIN_REPLY_FRAME_SIZE];

  /* Is this a legacy frame? If so continue. Otherwise check verion.*/
  if ((MRFI_GET_PAYLOAD_LEN(frame) - F_APP_PAYLOAD_OS) > JOIN_LEGACY_MSG_LENGTH)
  {
    /* see if protocol version is correct... */
    if (*(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+J_PROTOCOL_VERSION_OS) != nwk_getProtocolVersion())
    {
      /* Accommodation of protocol version differences can be noted or accomplished here.
       * Otherwise, no match and the board goes back
       */
      return;
    }
  }


  /* see if join token is correct */
  if (memcmp(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+J_JOIN_TOKEN_OS, &sJoinToken, sizeof(sJoinToken)))
  {
    /*TODO: maybe return a NAK error packet?? */
    return;
  }

  /* send reply with tid, the link token, and the encryption context */
  memcpy(msg+JR_LINK_TOKEN_OS, (void const *)&sLinkToken, sizeof(sLinkToken));
  msg[JR_CRYPTKEY_SIZE_OS] = SEC_CRYPT_KEY_SIZE;
  msg[JB_REQ_OS]           = JOIN_REQ_JOIN | NWK_APP_REPLY_BIT;
  /* sender's tid... */
  msg[JB_TID_OS]           = *(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+JB_TID_OS);

  if (pOutFrame = nwk_buildFrame(SMPL_PORT_JOIN, msg, sizeof(msg), MAX_HOPS_FROM_AP))
  {
    /* destination address is the source adddress of the received frame. */
    memcpy(MRFI_P_DST_ADDR(&pOutFrame->mrfiPkt), MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE);
#ifdef AP_IS_DATA_HUB
    /* if source device supports ED objects save source address to detect duplicate joins */
    if (*(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+J_NUMCONN_OS))
    {
      saveAddress(frame);
    }
#endif
  }
  else
  {
    /* oops -- no room left for Tx frame. Don't send reply. */
    return;
  }

  /* If this device polls we need to provide store-and-forward support */
  if (GET_FROM_FRAME(MRFI_P_PAYLOAD(frame),F_RX_TYPE) == F_RX_TYPE_POLLS)
  {
    /* save me! */
    if (sCurNumSandFClients < NUM_STORE_AND_FWD_CLIENTS)
    {
      uint8_t i;

      /* make sure it's not a duplicate */
      for (i=0; i<sCurNumSandFClients; ++i)
      {
        if (!memcmp(sSandFClients[i].addr, MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE))
        {
          break;
        }
      }
      if (i == sCurNumSandFClients)
      {
        /* save it -- it's not a duplicate */
        memcpy(sSandFClients[sCurNumSandFClients].addr, MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE);
        sCurNumSandFClients++;
      }
    }
    else
    {
      /* oops -- no room left. Don't send reply. */
      return;
    }
  }

  nwk_sendFrame(pOutFrame, MRFI_TX_TYPE_FORCED);

  return;
}

/******************************************************************************
 * @fn          nwk_join
 *
 * @brief       Stub Join function for Access Points.
 *
 * input parameters
 *
 * output parameters
 *
 * @return   Always returns SMPL_SUCCESS.
 */
smplStatus_t nwk_join(void)
{
  return SMPL_SUCCESS;
}

/******************************************************************************
 * @fn          nwk_isSandFClient
 *
 * @brief       Helper function to see if the destination of a frame we have is
 *              one of AP's store-and-forward clients.
 *
 * input parameters
 * @param   fPtr     - pointer to address in frame in question
 *
 * output parameters
 *
 * @return   Returns 1 if the destination is a store-and-forward client, else 0.
 */
uint8_t nwk_isSandFClient(uint8_t *pAddr)
{
  uint8_t i, end=sizeof(sSandFClients)/sizeof(sSandFClients[0]);

  for (i=0; i<end; ++i)
  {
    if (!memcmp(&sSandFClients[i], pAddr, NET_ADDR_SIZE))
    {
      return 1;
    }
  }

  return 0;
}

/******************************************************************************
 * @fn          nwk_setJoinContext
 *
 * @brief       Helper function to set Join context for Access Point. This will
 *              allow arbitration bewteen potentially nearby Access Points when
 *              a new device is joining.
 *
 * input parameters
 * @param   which   - Join context is either off or on
 *
 * output parameters
 *
 * @return   void
 */
void nwk_setJoinContext(uint8_t which)
{
  sJoinOK = (JOIN_CONTEXT_ON == which) ? 1 : 0;

  return;
}

/******************************************************************************
 * @fn          handleJoinRequest
 *
 * @brief       Dispatches handler for specfic join request
 *
 * input parameters
 *
 * @param   frame - Join frame received
 *
 * output parameters
 *
 * @return   void
 */
static void handleJoinRequest(mrfiPacket_t *frame)
{
  if (JOIN_LEGACY_MSG_LENGTH == (MRFI_GET_PAYLOAD_LEN(frame) - F_APP_PAYLOAD_OS))
  {
    /* Legacy frame. Spoof a join request */
    *(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS) = JOIN_REQ_JOIN;
  }

  switch (*(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS))
  {
    case JOIN_REQ_JOIN:
      smpl_send_join_reply(frame);
      break;

    default:
      break;
  }

  return;
}

#ifdef AP_IS_DATA_HUB
/******************************************************************************
 * @fn          saveAddress
 *
 * @brief       Helper function to save address of joined ED when the AP as
 *              data hub is in force. This will basically filters duplicate
 *              joins since the AP does not remember joined devices. It needs
 *              to is this case to save resources.
 *
 *              If this device is new execute the call back to that the AP
 *              application can do the SMPL_LinkListen().
 *
 * input parameters
 * @param   frame   - pointer to received join frame
 *
 * output parameters
 *
 * @return   void
 */
static void saveAddress(mrfiPacket_t *frame)
{
  if ((sNumJoined < NUM_CONNECTIONS) && !isJoined(frame))
  {
    memcpy(sJoinedED[sNumJoined].addr, MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE);
    sNumJoined++;
    if (spCallback)
    {
      spCallback(0);
    }
  }

  return;
}

/******************************************************************************
 * @fn          isJoined
 *
 * @brief       Helper function to look thorugh the list of joined devices to
 *              see if this one is new.
 *
 * input parameters
 * @param   frame   - pointer to received join frame
 *
 * output parameters
 *
 * @return   Returns '1' if the device has already joined, otherwise returns 0.
 */
static uint8_t isJoined(mrfiPacket_t *frame)
{
  uint8_t i;

  for (i=0; i<sNumJoined; ++i)
  {
    if (!memcmp(sJoinedED[i].addr, MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE))
    {
      return 1;
    }
  }

  return 0;
}

#endif /* AP_IS_DATA_HUB */

/******************************************************************************
 * @fn          nwk_purgeJoined
 *
 * @brief       Helper function to look thorugh the list of joined devices and
 *              purge the entry if found. This is necessary with the current
 *              mechanism for keeping track of already-joined devices. This
 *              implementation will change saving code and awkwardness.
 *
 *              Only needed when Unlink is supported.
 *
 * input parameters
 * @param   addr   - pointer to address to purge
 *
 * output parameters
 *
 * @return   void
 */
void nwk_purgeJoined(uint8_t *addr)
{
}
#else  /* ACCESS_POINT */

/******************************************************************************
 * @fn          nwk_join
 *
 * @brief       Join functioanlity for non-AP devices. Send the Join token
 *              and wait for the reply.
 *
 * input parameters
 *
 * output parameters
 *
 * @return   Status of operation.
 */
smplStatus_t nwk_join(void)
{
  uint8_t      msg[JOIN_FRAME_SIZE];
  uint32_t     linkToken;
  addr_t       apAddr;
  uint8_t      radioState = MRFI_GetRadioState();
  smplStatus_t rc = SMPL_NO_JOIN;
  union
  {
    ioctlRawSend_t    send;
    ioctlRawReceive_t recv;
  } ioctl_info;

#if defined( FREQUENCY_AGILITY )
  uint8_t  i, numChan;
  freqEntry_t channels[NWK_FREQ_TBL_SIZE];

  if (!(numChan=nwk_scanForChannels(channels)))
  {
    return SMPL_NO_CHANNEL;
  }

  for (i=0; i<numChan; ++i)
  {
    nwk_setChannel(&channels[i]);
#else
  {
#endif

    ioctl_info.send.addr = (addr_t *)nwk_getBCastAddress();
    ioctl_info.send.msg  = msg;
    ioctl_info.send.len  = sizeof(msg);
    ioctl_info.send.port = SMPL_PORT_JOIN;

    /* copy join token in */
    memcpy(msg+J_JOIN_TOKEN_OS, &sJoinToken, sizeof(sJoinToken));
    /* set app info byte */
    msg[JB_REQ_OS] = JOIN_REQ_JOIN;
    msg[JB_TID_OS] = sTid;
    /* Set number of connections supported. Used only by AP if it is
     * a data hub.
     */
    msg[J_NUMCONN_OS] = NUM_CONNECTIONS;
    /* protocol version number */
    msg[J_PROTOCOL_VERSION_OS] = nwk_getProtocolVersion();

    SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE, &ioctl_info.send);

    ioctl_info.recv.port = SMPL_PORT_JOIN;
    ioctl_info.recv.msg  = msg;
    ioctl_info.recv.addr = &apAddr;    /* save AP address from reply */

    NWK_CHECK_FOR_SETRX(radioState);
    NWK_REPLY_DELAY();
    NWK_CHECK_FOR_RESTORE_STATE(radioState);

    if (SMPL_SUCCESS == SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_READ, &ioctl_info.recv))
    {
      uint8_t firstByte = msg[JB_REQ_OS] & (~NWK_APP_REPLY_BIT);

      /* Sanity check for correct reply frame. Older version
       * has the length instead of the request as the first byte.
       */
      if ((firstByte == JOIN_REQ_JOIN) ||
          (firstByte == JOIN_REPLY_LEGACY_MSG_LENGTH)
         )
      {
        /* join reply returns link token */
        memcpy(&linkToken, msg+JR_LINK_TOKEN_OS, sizeof(linkToken));

        nwk_setLinkToken(linkToken);
        /* save AP address */
        nwk_setAPAddress(&apAddr);
        sTid++;   /* guard against duplicates */
        rc = SMPL_SUCCESS;
      }
    }
    /* TODO: process encryption stuff */
  }

  return rc;

}

#endif /* ACCESS_POINT */

/******************************************************************************
 * @fn          nwk_processJoin
 *
 * @brief       Processes a Join frame. If this is a reply let it go to the
 *              application. Otherwise generate and send the reply.
 *
 * input parameters
 * @param   frame     - Pointer to Join frame
 *
 * output parameters
 *
 * @return   Returns 1 if the frame should be saved in the Rx queue, else 0.
 */
fhStatus_t nwk_processJoin(mrfiPacket_t *frame)
{
  fhStatus_t rc = FHS_RELEASE;
  uint8_t    replyType;

  /* Make sure this is a reply and see if we sent this. Validate the
   * packet for reception by client app.
   */
  if (SMPL_MY_REPLY == (replyType=nwk_isValidReply(frame, sTid, JB_REQ_OS, JB_TID_OS)))
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
#if defined( ACCESS_POINT )
  else
  {
    /* send reply if we're an Access Point otherwise ignore the frame. */
    if ((SMPL_NOT_REPLY == replyType) && sJoinOK)
    {
      handleJoinRequest(frame);
    }
  }
#endif  /* ACCESS_POINT */
#endif  /* !END_DEVICE */

  (void) replyType;  /* keep compiler happy */

  return rc;
}





