
/**************************************************************************************************
  Filename:       nwk.c
  Revised:        $Date: 2008-07-02 15:37:50 -0700 (Wed, 02 Jul 2008) $
  Revision:       $Revision: 17454 $
  Author          $Author: lfriedman $

  Description:    This file supports the SimpliciTI network layer.


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
#include "nwk_frame.h"
#include "nwk.h"
#include "nwk_app.h"
#include "nwk_globals.h"
#include "nwk_QMgmt.h"

/******************************************************************************
 * MACROS
 */
/************************* NETWORK MANIFEST CONSTANT SANITY CHECKS ****************************/
#if !defined(ACCESS_POINT) && !defined(RANGE_EXTENDER) && !defined(END_DEVICE)
#error ERROR: No SimpliciTI device type defined
#endif

#if defined(END_DEVICE) && !defined(RX_POLLS)
#define RX_USER
#endif

#ifndef MAX_HOPS
#define MAX_HOPS  3
#elif MAX_HOPS > 4
#error ERROR: MAX_HOPS must be 4 or fewer
#endif

#ifndef MAX_APP_PAYLOAD
#error ERROR: MAX_APP_PAYLOAD must be defined
#endif

#if ( MAX_APP_PAYLOAD < MAX_FREQ_APP_FRAME )
#error ERROR: MAX_APP_PAYLOAD too small
#endif

#if ( MAX_APP_PAYLOAD < MAX_JOIN_APP_FRAME )
#error ERROR: MAX_APP_PAYLOAD too small
#endif

#if ( MAX_APP_PAYLOAD < MAX_LINK_APP_FRAME )
#error ERROR: MAX_APP_PAYLOAD too small
#endif

#if ( MAX_APP_PAYLOAD < MAX_MGMT_APP_FRAME )
#error ERROR: MAX_APP_PAYLOAD too small
#endif

#if ( MAX_APP_PAYLOAD < MAX_SEC_APP_FRAME )
#error ERROR: MAX_APP_PAYLOAD too small
#endif

#if ( MAX_APP_PAYLOAD < MAX_PING_APP_FRAME )
#error ERROR: MAX_APP_PAYLOAD too small
#endif

#if NWK_FREQ_TBL_SIZE < 1
#error ERROR: NWK_FREQ_TBL_SIZE must be > 0
#endif

/************************* END NETWORK MANIFEST CONSTANT SANITY CHECKS ************************/

/******************************************************************************
 * CONSTANTS AND DEFINES
 */
#define SYS_NUM_CONNECTIONS   (NUM_CONNECTIONS+1)

/* Increment this if the conTableInfo_t structure is changed. It will help
 * detect the upgrade context: any saved values will have a version with a
 * lower number.
 */
#define  CONNTABLEINFO_STRUCTURE_VERSION   1

/******************************************************************************
 * TYPEDEFS
 */
/* This structure aggregates eveything necessary to save if we want to restore
 * the connection information later.
 */
typedef struct
{
  const uint8_t    structureVersion; /* to dectect upgrades... */
        uint8_t    numConnections;   /* count includes the UUD port/link ID */
        connInfo_t connStruct[SYS_NUM_CONNECTIONS];
/* The next two are used to detect overlapping port assignments. When _sending_ a
 * link frame the local port is assigned from the top down. When sending a _reply_
 * the assignment is bottom up. Overlapping assignments are rejected. That said it
 * is extremely unlikely that this will ever happen. If it does the test implemented
 * here is overly cautious (it will reject assignments when it needn't). But we leave
 * it that way on the assumption that it will never happen anyway.
 */
        uint8_t    curNextLinkPort;
        uint8_t    curMaxReplyPort;
} conTableInfo_t;

/******************************************************************************
 * LOCAL VARIABLES
 */

/* This will be overwritten if we restore the structure from NV for example.
 * Note that restoring will not permit overwriting the version element as it
 * is declared 'const'.
 */
static conTableInfo_t sConTable = { CONNTABLEINFO_STRUCTURE_VERSION,
                                    SYS_NUM_CONNECTIONS,
                                    {0},
                                    SMPL_PORT_USER_MAX,
                                    PORT_BASE_NUMBER,
                                  };
static linkID_t  sNextLinkID = 1;

/******************************************************************************
 * LOCAL FUNCTIONS
 */
static uint8_t map_lid2idx(linkID_t, uint8_t *);

/******************************************************************************
 * GLOBAL VARIABLES
 */

/******************************************************************************
 * GLOBAL FUNCTIONS
 */

/******************************************************************************
 * @fn          nwk_nwkInit
 *
 * @brief       Initialize NWK conext.
 *
 * input parameters
 *
 * output parameters
 *
 * @return   Status of operation.
 */
smplStatus_t nwk_nwkInit(uint8_t (*f)(linkID_t))
{
  /* initialize globals */
  nwk_globalsInit();

  /* initialize frame processing */
  nwk_frameInit(f);

	/* added for CCE port */ 
  memset(sConTable.connStruct, 0, sizeof(sConTable.connStruct));
  nwk_QInit();
	
  /* initialize each network application. */
  nwk_freqInit();
  nwk_pingInit();
  nwk_joinInit(f);
  nwk_mgmtInit();
  nwk_linkInit();
  nwk_securityInit();

  /* set up the last connection as the broadcast port mapped to the broadcast Link ID */
  sConTable.connStruct[NUM_CONNECTIONS].isValid     = 1;
  sConTable.connStruct[NUM_CONNECTIONS].hops2target = MAX_HOPS;
  sConTable.connStruct[NUM_CONNECTIONS].portRx      = SMPL_PORT_USER_BCAST;
  sConTable.connStruct[NUM_CONNECTIONS].portTx      = SMPL_PORT_USER_BCAST;
  sConTable.connStruct[NUM_CONNECTIONS].thisLinkID  = SMPL_LINKID_USER_UUD;
  /* set peer address to broadcast so it is used when Application sends to the broadcast Link ID */
  memcpy(sConTable.connStruct[NUM_CONNECTIONS].peerAddr, nwk_getBCastAddress(), NET_ADDR_SIZE);

  return SMPL_SUCCESS;
}

/******************************************************************************
 * @fn          nwk_getNextConnection
 *
 * @brief       Return the next free connection structure if on is available.
 *
 * input parameters
 *
 * output parameters
 *      The returned structure has the Rx port number populated based on the
 *      free strucure found. This is the port queried when the app wants to
 *      do a receive.
 *
 * @return   pointer to the new connInfo_t structure. NULL if there is
 *           no room in connection structure array.
 */
connInfo_t *nwk_getNextConnection()
{
  uint8_t i;

  for (i=0; i<SYS_NUM_CONNECTIONS; ++i)
  {
    if (sConTable.connStruct[i].isValid)
    {
      continue;
    }
    break;
  }

  if (SYS_NUM_CONNECTIONS == i)
  {
    return (connInfo_t *)0;
  }

  /* this element will be populated during the exchange with the peer. */
  sConTable.connStruct[i].portTx = 0;

  sConTable.connStruct[i].isValid    = 1;
  sConTable.connStruct[i].thisLinkID = sNextLinkID;

  /* Generate the next Link ID. This isn't foolproof. If the count wraps
   * we can end up wthh confusing duplicates. We can protect aginst using
   * one that is already in use but we can't protect against a stale Link ID
   * remembered by an application that doesn't know its connection has been
   * torn down. The test for 0 will hopefully never be true (indicating a wrap).
   */
  sNextLinkID++;
  while (!sNextLinkID || (sNextLinkID == SMPL_LINKID_USER_UUD) || map_lid2idx(sNextLinkID, &i))
  {
    sNextLinkID++;
  }

  return &sConTable.connStruct[i];
}

/******************************************************************************
 * @fn          nwk_freeConnection
 *
 * @brief       Return the connection structure to the free pool. Currently
 *              this routine is only called when a link freame is sent and
 *              no reply is received so the freeing steps are pretty simple.
 *              But eventually this will be more complex so this place-holder
 *              is introduced.
 *
 * input parameters
 * @param   pCInfo    - pointer to entry to be freed
 *
 * output parameters
 *
 * @return   None.
 */
void nwk_freeConnection(connInfo_t *pCInfo)
{
#if NUM_CONNECTIONS > 0
  pCInfo->isValid = 0;
#endif
}

/******************************************************************************
 * @fn          nwk_getConnInfo
 *
 * @brief       Return the connection info structure to which the input Link ID maps.
 *
 * input parameters
 * @param   port    - port for which mapping desired
 *
 * output parameters
 *
 * @return   pointer to connInfo_t structure found. NULL if no mapping
 *           found or entry not valid.
 */
connInfo_t *nwk_getConnInfo(linkID_t linkID)
{
  uint8_t idx, rc;

  rc = map_lid2idx(linkID, &idx);

  return (rc && sConTable.connStruct[idx].isValid) ? &sConTable.connStruct[idx] : (connInfo_t *)0;
}

/******************************************************************************
 * @fn          nwk_isLinkDuplicate
 *
 * @brief       Help determine if the link has already been established.. Defense
 *              against duplicate link frames. This file owns the data structure
 *              so the comparison is done here.
 *
 * input parameters
 * @param   addr       - pointer to address of linker in question
 * @param   remotePort - remote port number provided by linker
 *
 * output parameters
 *
 * @return   Returns pointer to connection entry if the address and remote Port
 *           match an existing entry, otherwise 0.
 */
connInfo_t *nwk_isLinkDuplicate(uint8_t *addr, uint8_t remotePort)
{
#if NUM_CONNECTIONS > 0
  uint8_t       i;
  connInfo_t   *ptr = sConTable.connStruct;

  for (i=0; i<NUM_CONNECTIONS; ++i,++ptr)
  {
    if (ptr->isValid)
    {
      if (!(memcmp(ptr->peerAddr, addr, NET_ADDR_SIZE)) &&
          (ptr->portTx == remotePort))
      {
        return ptr;
      }
    }
  }

  return (connInfo_t *)0;
#else
  return (connInfo_t *)0;
#endif
}

/******************************************************************************
 * @fn          nwk_findAddressMatch
 *
 * @brief       Used to look for a match when a Tx-only device sends to the
 *              Tx-only port. If a match is found the local Rx port will be
 *              substituted for the Tx-only port in the frame so the local
 *              application can receive the payload "normally". Match is based
 *              on source address in frame.
 *
 * input parameters
 * @param   frame    - pointer to frame in question
 *
 * output parameters
 *
 * @return   Returns pointer to the matching connection info structure if
 *           a match is found, otherwise NULL.
 */
connInfo_t *nwk_findAddressMatch(mrfiPacket_t *frame)
{
#if NUM_CONNECTIONS > 0
  uint8_t       i;
  connInfo_t   *ptr = sConTable.connStruct;

  for (i=0; i<NUM_CONNECTIONS; ++i,++ptr)
  {
    if (ptr->isValid)
    {
      if (!(memcmp(ptr->peerAddr, MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE)))
      {
        return ptr;
      }
    }
  }

  return (connInfo_t *)0;
#else
  return (connInfo_t *)0;
#endif
}

/******************************************************************************
 * @fn          nwk_checkConnInfo
 *
 * @brief       Do a sanity/validity check on the connection info
 *
 * input parameters
 * @param   ptr     - pointer to a valid connection info structure to validate
 * @param   which   - Tx or Rx port checked
 *
 * output parameters
 *
 * @return   Status of operation.
 */
smplStatus_t nwk_checkConnInfo(connInfo_t *ptr, uint8_t which)
{
  uint8_t  port;

  /* make sure port isn't null and that the entry is active */
  port = (CHK_RX == which) ? ptr->portRx : ptr->portTx;
  if (!port || !ptr->isValid)
  {
    return SMPL_BAD_PARAM;
  }

  /* validate port number */
  if (port < PORT_BASE_NUMBER)
  {
    return SMPL_BAD_PARAM;
  }

  return SMPL_SUCCESS;
}

/******************************************************************************
 * @fn          nwk_isConnectionValid
 *
 * @brief       Do a sanity/validity check on the frame target address by
 *              validating frame against connection info
 *
 * input parameters
 * @param   frame   - pointer to frame in question
 *
 * output parameters
 * @param   lid   - link ID of found connection
 *
 * @return   0 if connection specified in frame is not valid, otherwise non-zero.
 */
uint8_t nwk_isConnectionValid(mrfiPacket_t *frame, linkID_t *lid)
{
  uint8_t       i;
  connInfo_t   *ptr  = sConTable.connStruct;
  uint8_t       port = GET_FROM_FRAME(MRFI_P_PAYLOAD(frame), F_PORT_OS);

  for (i=0; i<SYS_NUM_CONNECTIONS; ++i,++ptr)
  {
    if (ptr->isValid)
    {
      /* check port first since we're done if the port is the user bcast port. */
      if (port == ptr->portRx)
      {
        /* yep...ports match. */
        if ((SMPL_PORT_USER_BCAST == port) || !(memcmp(ptr->peerAddr, MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE)))
        {
          /* we're done. */
          *lid = ptr->thisLinkID;
          return 1;
        }
      }
    }
  }

  /* no matches */
  return 0;
}

/******************************************************************************
 * @fn          nwk_allocateLocalRxPort
 *
 * @brief       Allocate a local port on which to receive frames from a peer.
 *
 *              Allocation differs depending on whether the allocation is for
 *              a link reply frame or a link frame. In the former case we
 *              know the address of the peer so we can ensure allocating a
 *              unique port number for that address. The same port number can be
 *              used mulitple times for distinct peers. Allocations are done from
 *              the bottom of the namespace upward.
 *
 *              If allocation is for a link frame we do not yet know the peer
 *              address so we must ensure the port number is unique now.
 *              Allocations are done from the top of the namespace downward.
 *
 *              The two allocation methods track the extreme values used in each
 *              case to detect overlap, i.e., exhausted namespace. This can only
 *              happen if the number of connections supported is greater than the
 *              total namespace available.
 *
 * input parameters
 * @param   which   - Sending a link frame or a link reply frame
 * @param   newPtr  - pointer to connection info structure to be populated
 *
 * output parameters
 * @param   newPtr->portRx  - element is populated with port number.
 *
 * @return   Non-zero if port number assigned. 0 if no port available.
 */
uint8_t nwk_allocateLocalRxPort(uint8_t which, connInfo_t *newPtr)
{
#if NUM_CONNECTIONS > 0
  uint8_t     num, i;
  uint8_t     marker[NUM_CONNECTIONS] = {0};
  connInfo_t *ptr = sConTable.connStruct;

  for (i=0; i<NUM_CONNECTIONS; ++i,++ptr)
  {
    if ((ptr != newPtr) && ptr->isValid)
    {
      if (LINK_SEND == which)
      {
        if (ptr->portRx > sConTable.curNextLinkPort)
        {
          marker[SMPL_PORT_USER_MAX - ptr->portRx] = 1;
        }
      }
      else if (!memcmp(ptr->peerAddr, newPtr->peerAddr, NET_ADDR_SIZE))
      {
          marker[ptr->portRx - PORT_BASE_NUMBER] = 1;
      }
    }
  }

  num = 0;
  for (i=0; i<NUM_CONNECTIONS; ++i)
  {
    if (!marker[i])
    {
      if (LINK_REPLY == which)
      {
        num = PORT_BASE_NUMBER + i;
      }
      else
      {
        num = SMPL_PORT_USER_MAX - i;
      }
      break;
    }
  }

  if (LINK_REPLY == which)
  {
    /* if the number we have doesn't overlap the assignment of ports used
     * for sending link frames, use it.
     */
    if (num <= sConTable.curNextLinkPort)
    {
      if (num > sConTable.curMaxReplyPort)
      {
        /* remember maximum port number used */
        sConTable.curMaxReplyPort = num;
      }
    }
    else
    {
      /* the port number we need has already been used in the other context. It may or
       * may not have been used for the same address but we don't bother to check...we
       * just reject the asignment. This is the overly cautious part but is extermely
       * unlikely to ever occur.
       */
      num = 0;
    }
  }
  else
  {
    /* if the number we have doesn't overlap the assignment of ports used
     * for sending link frame replies, use it.
     */
    if (num >= sConTable.curMaxReplyPort)
    {
      if (num == sConTable.curNextLinkPort)
      {
        sConTable.curNextLinkPort--;
      }
    }
    else
    {
      /* the port number we need has already been used in the other context. It may or
       * may not have been used for the same address but we don't bother to check...we
       * just reject the asignment. This is the overly cautious part but is extermely
       * unlikely to ever occur.
       */
      num = 0;
    }
  }

  newPtr->portRx = num;

  return num;
#else
  return 0;
#endif  /* NUM_CONNECTIONS > 0 */

}

/*******************************************************************************
 * @fn          nwk_isValidReply
 *
 * @brief       Examine a frame to see if it is a valid reply when compared with
 *              expected parameters.
 *
 * input parameters
 * @param   frame      - pointer to frmae being examined
 * @param   tid        - expected transaction ID in application payload
 * @param   infoOffset - offset to payload information containing reply hint
 * @param   tidOffset  - offset to transaction ID in payload
 *
 * output parameters
 *
 * @return   reply category:
 *               SMPL_NOT_REPLY: not a reply
 *               SMPL_MY_REPLY : a reply that matches input parameters
 *               SMPL_A_REPLY  : a reply but does not match input parameters
 */
uint8_t nwk_isValidReply(mrfiPacket_t *frame, uint8_t tid, uint8_t infoOffset, uint8_t tidOffset)
{
  uint8_t rc = SMPL_NOT_REPLY;

  if ((*(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+infoOffset) & NWK_APP_REPLY_BIT))
  {
    if ((*(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+tidOffset) == tid) &&
        !memcmp(MRFI_P_DST_ADDR(frame), nwk_getMyAddress(), NET_ADDR_SIZE))
    {
      rc = SMPL_MY_REPLY;
    }
    else
    {
      rc = SMPL_A_REPLY;
    }
  }

  return rc;
}

/******************************************************************************
 * @fn          map_lid2idx
 *
 * @brief       Map link ID to index into connection table.
 *
 * input parameters
 * @param   lid   - Link ID to be matched
 *
 * output parameters
 * @param   idx   - populated with index into connection table
 *
 * @return   Non-zero if Link ID found and output is valid else 0.
 */
static uint8_t map_lid2idx(linkID_t lid, uint8_t *idx)
{
  uint8_t     i;
  connInfo_t *ptr = sConTable.connStruct;

  for (i=0; i<SYS_NUM_CONNECTIONS; ++i, ++ptr)
  {
    if (ptr->isValid && (ptr->thisLinkID == lid))
    {
      *idx = i;
      return 1;
    }
  }

  return 0;
}

/******************************************************************************
 * @fn          nwk_findPeer
 *
 * @brief       Find connection entry for a peer
 *
 * input parameters
 * @param   peerAddr   - address of peer
 * @param   peerPort   - port on which this device was sending to peer.
 *
 * output parameters
 *
 * @return   Pointer to matching connection table entry else 0.
 */
connInfo_t *nwk_findPeer(addr_t *peerAddr, uint8_t peerPort)
{
  uint8_t     i;
  connInfo_t *ptr = sConTable.connStruct;

  for (i=0; i<SYS_NUM_CONNECTIONS; ++i, ++ptr)
  {
    if (ptr->isValid)
    {
      if (!memcmp(peerAddr, ptr->peerAddr, NET_ADDR_SIZE))
      {
        if (peerPort == ptr->portTx)
        {
          return ptr;
        }
      }
    }
  }

  return 0;
}
