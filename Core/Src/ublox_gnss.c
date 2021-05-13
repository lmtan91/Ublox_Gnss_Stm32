/* *****************************************************************************
 *
 *
 ******************************************************************************/

/* *****************************************************************************
 * Files inclusion
 ******************************************************************************/
#include <ublox_gnss.h>
#include "timer.h"
/* *****************************************************************************
 * Internal Macros
 ******************************************************************************/


/* *****************************************************************************
 * Static function declaration
 ******************************************************************************/
static bool _printDebug = false;
static bool _printLimitedDebug = false;
/* *****************************************************************************
 * Variables
 ******************************************************************************/
uint8_t *payloadCfg_pu8 = NULL;
uint8_t *payloadAuto_pu8 = NULL;
uint8_t payloadAck[2];                // Holds the requested ACK/NACK
uint8_t payloadBuf[2];                // Temporary buffer used to screen incoming packets or dump unrequested packets
Ubx_Packet_tst packetCfg_st = {0, 0, 0, 0, 0, NULL, 0, 0, UBLOX_PACKET_VALIDITY_NOT_DEFINED, UBLOX_PACKET_VALIDITY_NOT_DEFINED};
Ubx_Packet_tst packetAck_st = {0, 0, 0, 0, 0, payloadAck, 0, 0, UBLOX_PACKET_VALIDITY_NOT_DEFINED, UBLOX_PACKET_VALIDITY_NOT_DEFINED};
Ubx_Packet_tst packetBuf_st = {0, 0, 0, 0, 0, payloadBuf, 0, 0, UBLOX_PACKET_VALIDITY_NOT_DEFINED, UBLOX_PACKET_VALIDITY_NOT_DEFINED};
Ubx_Packet_tst packetAuto_st = {0, 0, 0, 0, 0, NULL, 0, 0, UBLOX_PACKET_VALIDITY_NOT_DEFINED, UBLOX_PACKET_VALIDITY_NOT_DEFINED};

// Pointers to storage for the "automatic" messages
// RAM is allocated for these if/when required.
UBX_NAV_PVT_t *packetUBXNAVPVT_pst = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
//UBX_ESF_STATUS_t *gnss_pst->packetUBXESFSTATUS_pst = NULL;
moduleSWVersion_t *moduleSWVersion_pst = NULL;

uint8_t g_incomming = 0x00;

/* *****************************************************************************
 * Static function definition
 ******************************************************************************/



/* *****************************************************************************
 * Public function definition
 ******************************************************************************/
bool Gnss_Init(Ublox_Gnss_tst *gnss_pst, Serial_tst *serial_pst)
{
    bool connected = true;

    if(gnss_pst != NULL)
    {
        gnss_pst->currentSentence_e = NONE;
        gnss_pst->rtcmFrameCounter_u16 = 0;
        gnss_pst->ignoreThisPayload_b = false;
        gnss_pst->activePacketBuffer_e = UBLOX_PACKET_PACKETBUF;
        gnss_pst->packetCfgPayloadSize = 0;
        gnss_pst->packetUBXCFGRATE_pst = NULL;
        gnss_pst->commTypes_en = COMM_TYPE_SERIAL;
        gnss_pst->serial_pst = serial_pst;
        gnss_pst->checkCallbacksReentrant_b = false;
        gnss_pst->packetUBXESFSTATUS_pst = NULL;

        if(gnss_pst->packetCfgPayloadSize == 0)
        {
            Gnss_SetPacketCfgPayloadSize(gnss_pst, MAX_PAYLOAD_SIZE);
        }

        connected = Gnss_IsConnected(gnss_pst);
        if (!connected)
        {
            connected = Gnss_IsConnected(gnss_pst);
        }

        if (!connected)
        {
            connected = Gnss_IsConnected(gnss_pst);
        }
    }

    return connected;
}

bool Gnss_ConfMsg(struct Ublox_Gnss *gnss_pst, uint8_t msgClass_u8, uint8_t msgID_u8,
        uint8_t portID_u8, uint8_t sendRate_u8, uint16_t maxWait_u16)
{
    packetCfg_st.cls_u8 = UBX_CLASS_CFG;
    packetCfg_st.id_u8 = UBX_CFG_MSG;
    packetCfg_st.len_u16 = 2;
    packetCfg_st.startingSpot_u16 = 0;

    payloadCfg_pu8[0] = msgClass_u8;
    payloadCfg_pu8[1] = msgID_u8;

    /*This will load the payloadCfg_pu8 array with current settings of the given register*/
    // We are expecting data and an ACK
    if (Gnss_SendCmd(gnss_pst, &packetCfg_st, maxWait_u16, false) != UBLOX_STATUS_DATA_RECEIVED)
    {
        return (false);      /*If command send fails then bail*/
    }

    /*Now send it back with new mods*/
    packetCfg_st.len_u16 = 8;

    /*payloadCfg_pu8 is now loaded with current bytes. Change only the ones we need to
     *Send rate is relative to the event a message is registered on.
     *For example, if the rate of a navigation message is set to 2, the message is sent every 2nd navigation solution.
     */
    payloadCfg_pu8[2 + portID_u8] = sendRate_u8;

    /* We are only expecting an ACK */
    return ((Gnss_SendCmd(gnss_pst, &packetCfg_st, maxWait_u16, false)) == UBLOX_STATUS_DATA_SENT);
    return true;
}

void Gnss_SetPacketCfgPayloadSize(Ublox_Gnss_tst *gnss_pst, size_t payloadSz)
{
    if ((payloadSz == 0) && (payloadCfg_pu8 != NULL))
    {
      // Zero payloadSize? Dangerous! But we'll free the memory anyway...
      free(payloadCfg_pu8);
      payloadCfg_pu8 = NULL; // Redundant?
      packetCfg_st.payload_pu8 = payloadCfg_pu8;
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      {
          my_printf(("setPacketCfgPayloadSize: Zero payloadSize!\n"));
      }
    }
    else if (payloadCfg_pu8 == NULL) //Memory has not yet been allocated - so use new
    {
//      payloadCfg_pu8 = new uint8_t[payloadSize];
      payloadCfg_pu8 = (uint8_t *)malloc(payloadSz);
      packetCfg_st.payload_pu8 = payloadCfg_pu8;
      if (payloadCfg_pu8 == NULL)
      {
        if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        {
            my_printf("setPacketCfgPayloadSize: PANIC! RAM allocation failed!");
        }
      }
    }
    else //Memory has already been allocated - so resize
    {
//      uint8_t *newPayload = new uint8_t[payloadSize];
      uint8_t *newPayload = malloc(payloadSz);
      for (size_t i = 0; (i < payloadSz) && (i < gnss_pst->packetCfgPayloadSize); i++) // Copy as much existing data as we can
      {
        newPayload[i] = payloadCfg_pu8[i];
      }
      free(payloadCfg_pu8);
      payloadCfg_pu8 = newPayload;
      packetCfg_st.payload_pu8 = payloadCfg_pu8;
      if (payloadCfg_pu8 == NULL)
        if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
          my_printf("setPacketCfgPayloadSize: PANIC! RAM resize failed!");
    }

    gnss_pst->packetCfgPayloadSize = payloadSz;
}

ublox_status_ten Gnss_SendCmd(Ublox_Gnss_tst *gnss_pst, Ubx_Packet_tst *outgoingUBX_st, uint16_t maxWait_u16, bool expectACKonly_b)
{
    ublox_status_ten retVal = UBLOX_STATUS_SUCCESS;

    Gnss_CalcChecksum(gnss_pst, outgoingUBX_st); //Sets checksum A and B bytes of the packet

    if (_printDebug == true)
    {
      my_printf("\nSending: ");
      Gnss_PrintPacket(gnss_pst, outgoingUBX_st, true); // Always print payload
    }

    if (gnss_pst->commTypes_en == COMM_TYPE_SERIAL)
    {
        Gnss_SendSerialCommand(gnss_pst, outgoingUBX_st);
        /* TODO: should delay so that transmission is finished */
//        HAL_Delay(1);
    }

    if (maxWait_u16 > 0)
    {
      //Depending on what we just sent, either we need to look for an ACK or not
      if ((outgoingUBX_st->cls_u8 == UBX_CLASS_CFG) || (expectACKonly_b == true))
      {
        if (_printDebug == true)
        {
          my_printf("sendCommand: Waiting for ACK response");
        }
        //Wait for Ack response
        retVal = Gnss_WaitForACKResponse(gnss_pst, outgoingUBX_st, outgoingUBX_st->cls_u8, outgoingUBX_st->id_u8, maxWait_u16);
      }
      else
      {
        if (_printDebug == true)
        {
          my_printf("sendCommand: Waiting for No ACK response");
        }
        retVal = Gnss_WaitForNoACKResponse(gnss_pst, outgoingUBX_st, outgoingUBX_st->cls_u8, outgoingUBX_st->id_u8, maxWait_u16); //Wait for Ack response
      }
    }
    return retVal;
}

void Gnss_SendSerialCommand(struct Ublox_Gnss *gnss_pst, Ubx_Packet_tst *outgoingUBX_st)
{
    uint16_t index;
    uint8_t *pkg_pu8 = (uint8_t *)malloc(outgoingUBX_st->len_u16 + 8u);
    if(pkg_pu8 != NULL)
    {
        pkg_pu8[0] = UBX_SYNCH_1;
        pkg_pu8[1] = UBX_SYNCH_2;
        pkg_pu8[2] = outgoingUBX_st->cls_u8;
        pkg_pu8[3] = outgoingUBX_st->id_u8;
        pkg_pu8[4] = outgoingUBX_st->len_u16 & 0xFF;
        pkg_pu8[5] = outgoingUBX_st->len_u16 >> 8;
        for (index = 0; index < outgoingUBX_st->len_u16; index++)
        {
            pkg_pu8[index + 6u] = outgoingUBX_st->payload_pu8[index];
        }
        pkg_pu8[outgoingUBX_st->len_u16 + 8u - 2u] = outgoingUBX_st->checksumA_u8;
        pkg_pu8[outgoingUBX_st->len_u16 + 8u - 1u] = outgoingUBX_st->checksumB_u8;

        Serial_Write(gnss_pst->serial_pst, pkg_pu8, outgoingUBX_st->len_u16 + 8u);

        free(pkg_pu8);
    }
}

void Gnss_CalcChecksum(struct Ublox_Gnss *gnss_pst, Ubx_Packet_tst *msg_pst)
{
    msg_pst->checksumA_u8 = 0;
    msg_pst->checksumB_u8 = 0;

    msg_pst->checksumA_u8 += msg_pst->cls_u8;
    msg_pst->checksumB_u8 += msg_pst->checksumA_u8;

    msg_pst->checksumA_u8 += msg_pst->id_u8;
    msg_pst->checksumB_u8 += msg_pst->checksumA_u8;

    msg_pst->checksumA_u8 += (msg_pst->len_u16 & 0xFF);
    msg_pst->checksumB_u8 += msg_pst->checksumA_u8;

    msg_pst->checksumA_u8 += (msg_pst->len_u16 >> 8);
    msg_pst->checksumB_u8 += msg_pst->checksumA_u8;

    for (uint16_t i = 0; i < msg_pst->len_u16; i++)
    {
        msg_pst->checksumA_u8 += msg_pst->payload_pu8[i];
        msg_pst->checksumB_u8 += msg_pst->checksumA_u8;
    }
}

ublox_status_ten Gnss_WaitForACKResponse(struct Ublox_Gnss *gnss_pst, Ubx_Packet_tst *outgoingUBX_pst, uint8_t requestedClass_u8, uint8_t requestedID_u8, uint16_t maxTime_u16)
{
    outgoingUBX_pst->valid_en = UBLOX_PACKET_VALIDITY_NOT_DEFINED; //This will go VALID (or NOT_VALID) when we receive a response to the packet we sent
    packetAck_st.valid_en = UBLOX_PACKET_VALIDITY_NOT_DEFINED;
    packetBuf_st.valid_en = UBLOX_PACKET_VALIDITY_NOT_DEFINED;
    packetAuto_st.valid_en = UBLOX_PACKET_VALIDITY_NOT_DEFINED;
    outgoingUBX_pst->classAndIDmatch_en = UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a packet that matches the requested class and ID
    packetAck_st.classAndIDmatch_en = UBLOX_PACKET_VALIDITY_NOT_DEFINED;
    packetBuf_st.classAndIDmatch_en = UBLOX_PACKET_VALIDITY_NOT_DEFINED;
    packetAuto_st.classAndIDmatch_en = UBLOX_PACKET_VALIDITY_NOT_DEFINED;

    uint32_t startTime = timer_getCurrentTimeStampMs();
    uint8_t data_u8;

    do
    {
        if(Gnss_CheckUbloxInternal(gnss_pst, outgoingUBX_pst, requestedClass_u8, requestedID_u8) == true)
        {
            if ((outgoingUBX_pst->classAndIDmatch_en == UBLOX_PACKET_VALIDITY_VALID)
                    && (packetAck_st.classAndIDmatch_en == UBLOX_PACKET_VALIDITY_VALID)
                    && (outgoingUBX_pst->valid_en == UBLOX_PACKET_VALIDITY_VALID)
                    && (outgoingUBX_pst->cls_u8 == requestedClass_u8)
                    && (outgoingUBX_pst->id_u8 == requestedID_u8))
            {
                if (_printDebug == true)
                {
                    my_printf("waitForACKResponse: valid data and valid ACK received after ");
                }
                return (UBLOX_STATUS_DATA_RECEIVED); //We received valid data and a correct ACK!
            }
            else if ((outgoingUBX_pst->classAndIDmatch_en == UBLOX_PACKET_VALIDITY_NOT_DEFINED)
                    && (packetAck_st.classAndIDmatch_en == UBLOX_PACKET_VALIDITY_VALID))
            {
                if (_printDebug == true)
                {
                    my_printf("waitForACKResponse: no data and valid ACK after ");
                }
                return (UBLOX_STATUS_DATA_SENT); //We got an ACK but no data...
            }
            else if ((outgoingUBX_pst->classAndIDmatch_en == UBLOX_PACKET_VALIDITY_VALID)
                    && (packetAck_st.classAndIDmatch_en == UBLOX_PACKET_VALIDITY_VALID)
                    && ((outgoingUBX_pst->cls_u8 != requestedClass_u8) || (outgoingUBX_pst->id_u8 != requestedID_u8)))
            {
                if (_printDebug == true)
                {
                    my_printf("waitForACKResponse: data being OVERWRITTEN after ");
                }
                return (UBLOX_STATUS_DATA_OVERWRITTEN); // Data was valid but has been or is being overwritten
            }
            else if ((packetAck_st.classAndIDmatch_en == UBLOX_PACKET_VALIDITY_VALID)
                    && (outgoingUBX_pst->classAndIDmatch_en == UBLOX_PACKET_VALIDITY_NOT_VALID)
                    && (outgoingUBX_pst->valid_en == UBLOX_PACKET_VALIDITY_NOT_VALID))
            {
                if (_printDebug == true)
                {
                    my_printf("waitForACKResponse: CRC failed after ");
                }
                return (UBLOX_STATUS_CRC_FAIL); //Checksum fail
            }
            else if (packetAck_st.classAndIDmatch_en == UBLOX_PACKET_NOTACKNOWLEDGED)
            {
                if (_printDebug == true)
                {
                    my_printf("waitForACKResponse: data was NOTACKNOWLEDGED (NACK) after ");
                }
                return (UBLOX_STATUS_COMMAND_NACK); //We received a NACK!
            }
            else if ((outgoingUBX_pst->classAndIDmatch_en == UBLOX_PACKET_VALIDITY_VALID)
                    && (packetAck_st.classAndIDmatch_en == UBLOX_PACKET_VALIDITY_NOT_VALID)
                    && (outgoingUBX_pst->valid_en == UBLOX_PACKET_VALIDITY_VALID)
                    && (outgoingUBX_pst->cls_u8 == requestedClass_u8)
                    && (outgoingUBX_pst->id_u8 == requestedID_u8))
            {
                if (_printDebug == true)
                {
                    my_printf("waitForACKResponse: VALID data and INVALID ACK received after ");
                }
                return (UBLOX_STATUS_DATA_RECEIVED); //We received valid data and an invalid ACK!
            }
            else if ((outgoingUBX_pst->classAndIDmatch_en == UBLOX_PACKET_VALIDITY_NOT_VALID)
                    && (packetAck_st.classAndIDmatch_en == UBLOX_PACKET_VALIDITY_NOT_VALID))
            {
                if (_printDebug == true)
                {
                    my_printf("waitForACKResponse: INVALID data and INVALID ACK received after ");
                }
                return (UBLOX_STATUS_FAIL); //We received invalid data and an invalid ACK!
            }
            else if ((outgoingUBX_pst->classAndIDmatch_en == UBLOX_PACKET_VALIDITY_VALID)
                    && (packetAck_st.classAndIDmatch_en == UBLOX_PACKET_VALIDITY_NOT_DEFINED))
            {

            }

        } //checkUbloxInternal == true

        DWT_Delay(500);
    } while (timer_getTimeElapsed(startTime) < maxTime_u16);


    // We have timed out...
    // If the outgoingUBX_pst->classAndIDmatch_en is VALID then we can take a gamble and return DATA_RECEIVED
    // even though we did not get an ACK
    if ((outgoingUBX_pst->classAndIDmatch_en == UBLOX_PACKET_VALIDITY_VALID) && (packetAck_st.classAndIDmatch_en == UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (outgoingUBX_pst->valid_en == UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX_pst->cls_u8 == requestedClass_u8) && (outgoingUBX_pst->id_u8 == requestedID_u8))
    {
      if (_printDebug == true)
      {
        my_printf(("waitForACKResponse: TIMEOUT with valid_en data after "));
      }
      return (UBLOX_STATUS_DATA_RECEIVED); //We received valid_en data... But no ACK!
    }

    if (_printDebug == true)
    {
      my_printf(("waitForACKResponse: TIMEOUT after "));
    }

    return (UBLOX_STATUS_TIMEOUT);
}

ublox_status_ten Gnss_WaitForNoACKResponse(struct Ublox_Gnss *gnss_pst, Ubx_Packet_tst *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime)
{
    outgoingUBX->valid_en = UBLOX_PACKET_VALIDITY_NOT_DEFINED; //This will go VALID (or NOT_VALID) when we receive a response to the packet we sent
    packetAck_st.valid_en = UBLOX_PACKET_VALIDITY_NOT_DEFINED;
    packetBuf_st.valid_en = UBLOX_PACKET_VALIDITY_NOT_DEFINED;
    packetAuto_st.valid_en = UBLOX_PACKET_VALIDITY_NOT_DEFINED;
    outgoingUBX->classAndIDmatch_en = UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a packet that matches the requested class and ID
    packetAck_st.classAndIDmatch_en = UBLOX_PACKET_VALIDITY_NOT_DEFINED;
    packetBuf_st.classAndIDmatch_en = UBLOX_PACKET_VALIDITY_NOT_DEFINED;
    packetAuto_st.classAndIDmatch_en = UBLOX_PACKET_VALIDITY_NOT_DEFINED;

    unsigned long startTime = HAL_GetTick();
    while (HAL_GetTick() - startTime < maxTime)
    {
      if (Gnss_CheckUbloxInternal(gnss_pst, outgoingUBX, requestedClass, requestedID) == true) //See if new data is available. Process bytes as they come in.
      {
        if ((outgoingUBX->classAndIDmatch_en == UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->valid_en == UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls_u8 == requestedClass) && (outgoingUBX->id_u8 == requestedID))
        {
          if (_printDebug == true)
          {
            my_printf(("waitForNoACKResponse: valid_en data with CLS/ID match after "));
          }
          return (UBLOX_STATUS_DATA_RECEIVED); //We received valid_en data!
        }
        else if ((outgoingUBX->classAndIDmatch_en == UBLOX_PACKET_VALIDITY_VALID) && ((outgoingUBX->cls_u8 != requestedClass) || (outgoingUBX->id_u8 != requestedID)))
        {
          if (_printDebug == true)
          {
            my_printf(("waitForNoACKResponse: data being OVERWRITTEN after "));
          }
          return (UBLOX_STATUS_DATA_OVERWRITTEN); // Data was valid_en but has been or is being overwritten
        }

        // If outgoingUBX->classAndIDmatch_en is NOT_DEFINED
        // and outgoingUBX->valid_en is VALID then this must be (e.g.) a PVT packet
        else if ((outgoingUBX->classAndIDmatch_en == UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (outgoingUBX->valid_en == UBLOX_PACKET_VALIDITY_VALID))
        {
           if (_printDebug == true)
           {
             my_printf(("waitForNoACKResponse: valid_en but UNWANTED data after "));
          //   my_printf(millis() - startTime);
          //   my_printf((" msec. Class: "));
          //   my_printf(outgoingUBX->cls_u8);
          //   my_printf((" ID: "));
          //   my_printf(outgoingUBX->id_u8);
           }
        }

        // If the outgoingUBX->classAndIDmatch_en is NOT_VALID then we return CRC failure
        else if (outgoingUBX->classAndIDmatch_en == UBLOX_PACKET_VALIDITY_NOT_VALID)
        {
          if (_printDebug == true)
          {
            my_printf(("waitForNoACKResponse: CLS/ID match but failed CRC after "));
//            my_printf(millis() - startTime);
//            my_printf(F(" msec"));
          }
          return (UBLOX_STATUS_CRC_FAIL); //We received invalid data
        }
      }

      DWT_Delay(500);
    }
    if (_printDebug == true)
    {
      my_printf(("waitForNoACKResponse: TIMEOUT after "));
//      my_printf(millis() - startTime);
//      my_printf(F(" msec. No packet received."));
    }
    return (UBLOX_STATUS_TIMEOUT);

}

void Gnss_CheckCallbacks(struct Ublox_Gnss *gnss_pst)
{
    if (gnss_pst->checkCallbacksReentrant_b == true) // Check for reentry (i.e. checkCallbacks has been called from inside a callback)
    {
        return;
    }
    gnss_pst->checkCallbacksReentrant_b = true;

    if ((packetUBXNAVPVT_pst != NULL) // If RAM has been allocated for message storage
      && (packetUBXNAVPVT_pst->callbackData != NULL) // If RAM has been allocated for the copy of the data
      && (packetUBXNAVPVT_pst->callbackPointer != NULL) // If the pointer to the callback has been defined
      && (packetUBXNAVPVT_pst->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
    {
        packetUBXNAVPVT_pst->callbackPointer(*packetUBXNAVPVT_pst->callbackData); // Call the callback
        packetUBXNAVPVT_pst->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
    }

    gnss_pst->checkCallbacksReentrant_b = false;
}

bool Gnss_EnableNMEAMessage(struct Ublox_Gnss *gnss_pst, uint8_t msgID_u8, uint8_t portID_u8, uint8_t sendRate_u8, uint16_t maxWait_u16)
{
    return (Gnss_ConfMsg(gnss_pst, UBX_CLASS_NMEA, msgID_u8, portID_u8, sendRate_u8, maxWait_u16));
}

bool Gnss_DisableNMEAMessage(struct Ublox_Gnss *gnss_pst, uint8_t msgID_u8, uint8_t portID_u8, uint16_t maxWait_u16)
{
    return (Gnss_EnableNMEAMessage(gnss_pst, msgID_u8, portID_u8, 0, maxWait_u16));
}

bool Gnss_IsConnected(struct Ublox_Gnss *gnss_pst)
{
    return Gnss_IsConnectedT(gnss_pst, 1100);
}

bool Gnss_IsConnectedT(struct Ublox_Gnss *gnss_pst, uint16_t maxWait_u16)
{
    return Gnss_GetNavigationFrequencyInternal(gnss_pst, maxWait_u16);
}

bool Gnss_InitPacketUBXCFGRATE(Ublox_Gnss_tst *gnss_pst)
{
    gnss_pst->packetUBXCFGRATE_pst = (UBX_CFG_RATE_tst *)malloc(sizeof(UBX_CFG_RATE_tst));
    if (gnss_pst->packetUBXCFGRATE_pst == NULL)
    {
        return (false);
    }
    gnss_pst->packetUBXCFGRATE_pst->automaticFlags.flags.all = 0;
    gnss_pst->packetUBXCFGRATE_pst->callbackPointer = NULL;
    gnss_pst->packetUBXCFGRATE_pst->callbackData = NULL;
    gnss_pst->packetUBXCFGRATE_pst->moduleQueried.moduleQueried.all = 0;
    return (true);
}

bool Gnss_GetNavigationFrequencyInternal(struct Ublox_Gnss *gnss_pst, uint16_t maxWait_u16)
{
    if (gnss_pst->packetUBXCFGRATE_pst == NULL)
    {
        Gnss_InitPacketUBXCFGRATE(gnss_pst);
        if (gnss_pst->packetUBXCFGRATE_pst == NULL)
        {
            return false;
        }
    }

    if (gnss_pst->packetUBXCFGRATE_pst->automaticFlags.flags.bits.automatic && gnss_pst->packetUBXCFGRATE_pst->automaticFlags.flags.bits.implicitUpdate)
    {
      //The GPS is automatically reporting, we just check whether we got unread data
        Gnss_CheckUbloxInternal(gnss_pst, &packetCfg_st, UBX_CLASS_CFG, UBX_CFG_RATE);
      return gnss_pst->packetUBXCFGRATE_pst->moduleQueried.moduleQueried.bits.all;
    }
    else if (gnss_pst->packetUBXCFGRATE_pst->automaticFlags.flags.bits.automatic && !gnss_pst->packetUBXCFGRATE_pst->automaticFlags.flags.bits.implicitUpdate)
    {
      //Someone else has to call checkUblox for us...
      return (false);
    }
    else
    {
      //The GPS is not automatically reporting navigation rate so we have to poll explicitly
      packetCfg_st.cls_u8 = UBX_CLASS_CFG;
      packetCfg_st.id_u8 = UBX_CFG_RATE;
      packetCfg_st.len_u16 = 0;
      packetCfg_st.startingSpot_u16 = 0;

      //The data is parsed as part of processing the response
      ublox_status_ten retVal = Gnss_SendCmd(gnss_pst, &packetCfg_st, maxWait_u16, false);

      if (retVal == UBLOX_STATUS_DATA_RECEIVED)
      {
          return (true);
      }

      if (retVal == UBLOX_STATUS_DATA_OVERWRITTEN)
      {
          return (true);
      }

      return (false);
    }
}

bool Gnss_CheckUblox(struct Ublox_Gnss *gnss_pst, uint8_t requestedClass_u8, uint8_t requestedID_u8)
{
    return Gnss_CheckUbloxInternal(gnss_pst, &packetCfg_st, requestedClass_u8, requestedID_u8);
}

bool Gnss_CheckUbloxInternal(struct Ublox_Gnss *gnss_pst, Ubx_Packet_tst *incomingUBX_pst, uint8_t requestedClass_u8, uint8_t requestedID_u8)
{
    if (gnss_pst->commTypes_en == COMM_TYPE_SERIAL)
    {
        return (Gnss_CheckUbloxSerial(gnss_pst, incomingUBX_pst, requestedClass_u8, requestedID_u8));
    }
    return false;
}

bool Gnss_CheckUbloxSerial(struct Ublox_Gnss *gnss_pst, Ubx_Packet_tst *incomingUBX_pst, uint8_t requestedClass_u8, uint8_t requestedID_u8)
{
    uint8_t data_u8;

    while( Serial_Available(gnss_pst->serial_pst) )
    {
        Serial_Read(gnss_pst->serial_pst, &data_u8);

        Gnss_Process(gnss_pst, data_u8, incomingUBX_pst, requestedClass_u8, requestedID_u8);
    }

    return (true);
}

bool Gnss_GetPortSettings(struct Ublox_Gnss *gnss_pst, uint8_t portID_u8, uint16_t maxWait_u16)
{
    packetCfg_st.cls_u8 = UBX_CLASS_CFG;
    packetCfg_st.id_u8 = UBX_CFG_PRT;
    packetCfg_st.len_u16 = 1;
    packetCfg_st.startingSpot_u16 = 0;

    payloadCfg_pu8[0] = portID_u8;

    return ((Gnss_SendCmd(gnss_pst, &packetCfg_st, maxWait_u16, false)) == UBLOX_STATUS_DATA_RECEIVED); // We are expecting data and an ACK
}

bool Gnss_SetPortOutput(struct Ublox_Gnss *gnss_pst, uint8_t portID, uint8_t outStreamSettings, uint16_t maxWait)
{
    //Get the current config values for this port ID
     if (Gnss_GetPortSettings(gnss_pst, portID, maxWait) == false)
     {
       return (false);
     }

     packetCfg_st.cls_u8 = UBX_CLASS_CFG;
     packetCfg_st.id_u8 = UBX_CFG_PRT;
     packetCfg_st.len_u16 = 20;
     packetCfg_st.startingSpot_u16 = 0;

     //payloadCfg is now loaded with current bytes. Change only the ones we need to
     payloadCfg_pu8[14] = outStreamSettings; //OutProtocolMask LSB - Set outStream bits

     return ((Gnss_SendCmd(gnss_pst, &packetCfg_st, maxWait, false)) == UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

bool Gnss_SetUART1Output(struct Ublox_Gnss *gnss_pst, uint8_t comSettings, uint16_t maxWait)
{
    return (Gnss_SetPortOutput(gnss_pst, COM_PORT_UART1, comSettings, maxWait));
}

bool Gnss_SetUART2Output(struct Ublox_Gnss *gnss_pst, uint8_t comSettings, uint16_t maxWait)
{
    return (Gnss_SetPortOutput(gnss_pst, COM_PORT_UART2, comSettings, maxWait));
}


void Gnss_ProcessUBX(struct Ublox_Gnss *gnss_pst, uint8_t incoming_u8, Ubx_Packet_tst *incomingUBX_pst, uint8_t requestedClass_u8, uint8_t requestedID_u8)
{
   size_t maximum_payload_size;
   if (gnss_pst->activePacketBuffer_e == UBLOX_PACKET_PACKETCFG)
   {
       maximum_payload_size = gnss_pst->packetCfgPayloadSize;
   }
   else if (gnss_pst->activePacketBuffer_e == UBLOX_PACKET_PACKETAUTO)
   {
       maximum_payload_size = Gnss_GetMaxPayloadSize(gnss_pst, incomingUBX_pst->cls_u8, incomingUBX_pst->id_u8);
       if (maximum_payload_size == 0)
       {
         if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
         {
           my_printf("processUBX: getMaxPayloadSize returned ZERO!! Class: 0x%02x ID: 0x%02X", incomingUBX_pst->cls_u8, incomingUBX_pst->id_u8);
         }
       }
     //}
     //else
     //  maximum_payload_size = 2;
   }
   else
   {
     maximum_payload_size = 2;
   }

   bool overrun = false;

   //Add all incoming bytes to the rolling checksum
   //Stop at len_u16+4 as this is the checksum bytes to that should not be added to the rolling checksum
   if (incomingUBX_pst->counter_u16 < incomingUBX_pst->len_u16 + 4)
   {
     Gnss_AddToChecksum(gnss_pst, incoming_u8);
   }

   if (incomingUBX_pst->counter_u16 == 0)
   {
     incomingUBX_pst->cls_u8 = incoming_u8;
   }
   else if (incomingUBX_pst->counter_u16 == 1)
   {
       incomingUBX_pst->id_u8 = incoming_u8;
   }
   else if (incomingUBX_pst->counter_u16 == 2) //Len LSB
   {
       incomingUBX_pst->len_u16 = incoming_u8;
   }
   else if (incomingUBX_pst->counter_u16 == 3) //Len MSB
   {
       incomingUBX_pst->len_u16 |= incoming_u8 << 8;
   }
   else if (incomingUBX_pst->counter_u16 == incomingUBX_pst->len_u16 + 4) //ChecksumA
   {
       incomingUBX_pst->checksumA_u8 = incoming_u8;
   }
   else if (incomingUBX_pst->counter_u16 == incomingUBX_pst->len_u16 + 5) //ChecksumB
   {
       incomingUBX_pst->checksumB_u8 = incoming_u8;

     gnss_pst->currentSentence_e = NONE; //We're done! Reset the sentence to being looking for a new start char

     //Validate this sentence
     if ((incomingUBX_pst->checksumA_u8 == gnss_pst->rollingChecksumA_u8)
             && (incomingUBX_pst->checksumB_u8 == gnss_pst->rollingChecksumB_u8))
     {
       incomingUBX_pst->valid_en = UBLOX_PACKET_VALIDITY_VALID; // Flag the packet as valid

       // Let's check if the class and ID match the requestedClass and requestedID
       // Remember - this could be a data packet or an ACK packet
       if ((incomingUBX_pst->cls_u8 == requestedClass_u8) && (incomingUBX_pst->id_u8 == requestedID_u8))
       {
         incomingUBX_pst->classAndIDmatch_en = UBLOX_PACKET_VALIDITY_VALID; // If we have a match, set the classAndIDmatch flag to valid
       }
       // If this is an ACK then let's check if the class and ID match the requestedClass and requestedID
       else if ((incomingUBX_pst->cls_u8 == UBX_CLASS_ACK)
               && (incomingUBX_pst->id_u8 == UBX_ACK_ACK)
               && (incomingUBX_pst->payload_pu8[0] == requestedClass_u8) && (incomingUBX_pst->payload_pu8[1] == requestedID_u8))
       {
         incomingUBX_pst->classAndIDmatch_en = UBLOX_PACKET_VALIDITY_VALID; // If we have a match, set the classAndIDmatch flag to valid
       }

       // If this is a NACK then let's check if the class and ID match the requestedClass and requestedID
       else if ((incomingUBX_pst->cls_u8 == UBX_CLASS_ACK) && (incomingUBX_pst->id_u8 == UBX_ACK_NACK)
               && (incomingUBX_pst->payload_pu8[0] == requestedClass_u8) && (incomingUBX_pst->payload_pu8[1] == requestedID_u8))
       {
           incomingUBX_pst->classAndIDmatch_en = UBLOX_PACKET_NOTACKNOWLEDGED; // If we have a match, set the classAndIDmatch flag to NOTACKNOWLEDGED
         if (_printDebug == true)
         {
           my_printf("processUBX: NACK received: Requested Class: 0x%02x Requested ID: 0x%02x",
                   incomingUBX_pst->payload_pu8[0], incomingUBX_pst->payload_pu8[1]);
         }
       }

       //This is not an ACK and we do not have a complete class and ID match
       //So let's check for an "automatic" message arriving
       else if (Gnss_CheckAutomatic(gnss_pst, incomingUBX_pst->cls_u8, incomingUBX_pst->id_u8))
       {
         // This isn't the message we are looking for...
         // Let's say so and leave incomingUBX->classAndIDmatch _unchanged_
         if (_printDebug == true)
         {
           my_printf("processUBX: incoming \"automatic\" message: Class: 0x%02x ID: 0x", incomingUBX_pst->cls_u8, incomingUBX_pst->id_u8);
         }
       }

       if (_printDebug == true)
       {
           my_printf("Incoming: Size: %d Received: ", incomingUBX_pst->len_u16);

           Gnss_PrintPacket(gnss_pst, incomingUBX_pst, false);

         if (incomingUBX_pst->valid_en == UBLOX_PACKET_VALIDITY_VALID)
         {
           my_printf("packetCfg now valid");
         }
         if (packetAck_st.valid_en == UBLOX_PACKET_VALIDITY_VALID)
         {
             my_printf("packetAck now valid");
         }
         if (incomingUBX_pst->classAndIDmatch_en == UBLOX_PACKET_VALIDITY_VALID)
         {
             my_printf("packetCfg classAndIDmatch");
         }
         if (packetAck_st.classAndIDmatch_en == UBLOX_PACKET_VALIDITY_VALID)
         {
             my_printf("packetAck classAndIDmatch");
         }
       }

       //We've got a valid packet, now do something with it but only if ignoreThisPayload is false
       if (gnss_pst->ignoreThisPayload_b == false)
       {
           Gnss_ProcessUBXpacket(gnss_pst, incomingUBX_pst);
       }
     }
     else // Checksum failure
     {
       incomingUBX_pst->valid_en = UBLOX_PACKET_VALIDITY_NOT_VALID;

       // Let's check if the class and ID match the requestedClass and requestedID.
       // This is potentially risky as we are saying that we saw the requested Class and ID
       // but that the packet checksum failed. Potentially it could be the class or ID bytes
       // that caused the checksum error!
       if ((incomingUBX_pst->cls_u8 == requestedClass_u8) && (incomingUBX_pst->id_u8 == requestedID_u8))
       {
         incomingUBX_pst->classAndIDmatch_en = UBLOX_PACKET_VALIDITY_NOT_VALID; // If we have a match, set the classAndIDmatch flag to not valid
       }
       // If this is an ACK then let's check if the class and ID match the requestedClass and requestedID
       else if ((incomingUBX_pst->cls_u8 == UBX_CLASS_ACK) && (incomingUBX_pst->payload_pu8[0] == requestedClass_u8)
               && (incomingUBX_pst->payload_pu8[1] == requestedID_u8))
       {
         incomingUBX_pst->classAndIDmatch_en = UBLOX_PACKET_VALIDITY_NOT_VALID; // If we have a match, set the classAndIDmatch flag to not valid
       }

       if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
       {
         //Drive an external pin to allow for easier logic analyzation
//         if (debugPin >= 0)
//         {
//           digitalWrite((uint8_t)debugPin, LOW);
//           delay(10);
//           digitalWrite((uint8_t)debugPin, HIGH);
//         }

         my_printf("Checksum failed: checksumA: %02x checksumB: %02x rollingChecksumA: 0x%02x rollingChecksumB: 0x%02x",
                 incomingUBX_pst->checksumA_u8, incomingUBX_pst->checksumB_u8,
                 gnss_pst->rollingChecksumA_u8, gnss_pst->rollingChecksumB_u8);
       }
     }

     // Now that the packet is complete and has been processed, we need to delete the memory
     // allocated for packetAuto
     if (gnss_pst->activePacketBuffer_e == UBLOX_PACKET_PACKETAUTO)
     {
       if(payloadAuto_pu8 != NULL)
       {
           free(payloadAuto_pu8);
           payloadAuto_pu8 = NULL; // Redundant?
           packetAuto_st.payload_pu8 = payloadAuto_pu8;
       }
     }
   }
   else //Load this byte into the payload array
   {
     //If an automatic packet comes in asynchronously, we need to fudge the startingSpot
     uint16_t startingSpot = incomingUBX_pst->startingSpot_u16;
     if (Gnss_CheckAutomatic(gnss_pst, incomingUBX_pst->cls_u8, incomingUBX_pst->id_u8))
     {
       startingSpot = 0;
     }
     // Check if this is payload data which should be ignored
     if (gnss_pst->ignoreThisPayload_b == false)
     {
       //Begin recording if counter goes past startingSpot
       if ((incomingUBX_pst->counter_u16 - 4) >= startingSpot)
       {
         //Check to see if we have room for this byte
         if (((incomingUBX_pst->counter_u16 - 4) - startingSpot) < maximum_payload_size) //If counter = 208, starting spot = 200, we're good to record.
         {
           incomingUBX_pst->payload_pu8[(incomingUBX_pst->counter_u16 - 4) - startingSpot] = incoming_u8; //Store this byte into payload array
         }
         else
         {
           overrun = true;
         }
       }
     }
   }

   // incomingUBX->counter should never reach maximum_payload_size + class + id_u8 + len_u16[2] + checksum[2]
   if (overrun || ((incomingUBX_pst->counter_u16 == maximum_payload_size + 6) && (gnss_pst->ignoreThisPayload_b == false)))
   {
     //Something has gone very wrong
       gnss_pst->currentSentence_e = NONE; //Reset the sentence to being looking for a new start char
     if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
     {
       if (overrun)
       {
         my_printf("processUBX: buffer overrun detected!");
       }
       else
       {
         my_printf("processUBX: counter hit maximum_payload_size + 6! activePacketBuffer: %d maximum_payload_size: %d",
                 gnss_pst->activePacketBuffer_e, maximum_payload_size);
       }
     }
   }

   //Increment the counter
   incomingUBX_pst->counter_u16++;

}

void Gnss_Process(struct Ublox_Gnss *gnss_pst, uint8_t incoming_u8, Ubx_Packet_tst *incomingUBX_pst, uint8_t requestedClass_u8, uint8_t requestedID_u8)
{
    if ((gnss_pst->currentSentence_e == NONE) || (gnss_pst->currentSentence_e == NMEA))
    {
        if (incoming_u8 == 0xB5) //UBX binary frames start with 0xB5, aka μ
        {
          //This is the start of a binary sentence. Reset flags.
          //We still don't know the response class
            gnss_pst->ubxFrameCounter_u16 = 0;
            gnss_pst->currentSentence_e = UBX;
            //Reset the packetBuf.counter even though we will need to reset it again when ubxFrameCounter == 2
            packetBuf_st.counter_u16 = 0;
            gnss_pst->ignoreThisPayload_b = false; //We should not ignore this payload - yet
          //Store data in packetBuf until we know if we have a requested class and ID match
            gnss_pst->activePacketBuffer_e = UBLOX_PACKET_PACKETBUF;
        }
        else if (incoming_u8 == '$')
        {
            gnss_pst->currentSentence_e = NMEA;
        }
        else if (incoming_u8 == 0xD3) //RTCM frames start with 0xD3
        {
            gnss_pst->rtcmFrameCounter_u16 = 0;
          gnss_pst->currentSentence_e = RTCM;
        }
        else
        {
          //This character is unknown or we missed the previous start of a sentence
        }
    }

    //Depending on the sentence, pass the character to the individual processor
    if (gnss_pst->currentSentence_e == UBX)
    {
        //Decide what type of response this is
        if ((gnss_pst->ubxFrameCounter_u16 == 0) && (incoming_u8 != 0xB5))      //ISO 'μ'
        {
            gnss_pst->currentSentence_e = NONE;                              //Something went wrong. Reset.
        }
        else if ((gnss_pst->ubxFrameCounter_u16 == 1) && (incoming_u8 != 0x62)) //ASCII 'b'
        {
            gnss_pst->currentSentence_e = NONE;                              //Something went wrong. Reset.
        }
        // Note to future self:
        // There may be some duplication / redundancy in the next few lines as processUBX will also
        // load information into packetBuf, but we'll do it here too for clarity
        else if (gnss_pst->ubxFrameCounter_u16 == 2) //Class
        {
          // Record the class in packetBuf until we know what to do with it
          packetBuf_st.cls_u8 = incoming_u8; // (Duplication)
          gnss_pst->rollingChecksumA_u8 = 0;     //Reset our rolling checksums here (not when we receive the 0xB5)
          gnss_pst->rollingChecksumB_u8 = 0;
          packetBuf_st.counter_u16 = 0;                                   //Reset the packetBuf.counter (again)
          packetBuf_st.valid_en = UBLOX_PACKET_VALIDITY_NOT_DEFINED; // Reset the packet validity (redundant?)
          packetBuf_st.startingSpot_u16 = incomingUBX_pst->startingSpot_u16;      //Copy the startingSpot
        }
        else if (gnss_pst->ubxFrameCounter_u16 == 3) //ID
        {
            // Record the ID in packetBuf until we know what to do with it
            packetBuf_st.id_u8 = incoming_u8; // (Duplication)
            //We can now identify the type of response
            //If the packet we are receiving is not an ACK then check for a class and ID match
            if (packetBuf_st.cls_u8 != UBX_CLASS_ACK)
            {
                //This is not an ACK so check for a class and ID match
                if ((packetBuf_st.cls_u8 == requestedClass_u8) && (packetBuf_st.id_u8 == requestedID_u8))
                {
                    //This is not an ACK and we have a class and ID match
                    //So start diverting data into incomingUBX (usually packetCfg)
                    gnss_pst->activePacketBuffer_e = UBLOX_PACKET_PACKETCFG;
                    incomingUBX_pst->cls_u8 = packetBuf_st.cls_u8; //Copy the class and ID into incomingUBX (usually packetCfg)
                    incomingUBX_pst->id_u8 = packetBuf_st.id_u8;
                    incomingUBX_pst->counter_u16 = packetBuf_st.counter_u16; //Copy over the .counter too
                }
                //This is not an ACK and we do not have a complete class and ID match
                //So let's check if this is an "automatic" message which has its own storage defined
                else if (Gnss_CheckAutomatic(gnss_pst, packetBuf_st.cls_u8, packetBuf_st.id_u8))
                {
                    //This is not the message we were expecting but it has its own storage and so we should process it anyway.
                    //We'll try to use packetAuto to buffer the message (so it can't overwrite anything in packetCfg).
                    //We need to allocate memory for the packetAuto payload (payloadAuto) - and delete it once
                    //reception is complete.
                    uint16_t maxPayload = Gnss_GetMaxPayloadSize(gnss_pst, packetBuf_st.cls_u8, packetBuf_st.id_u8); // Calculate how much RAM we need
                    if (maxPayload == 0)
                    {
                        if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
                        {
                            my_printf(("process: getMaxPayloadSize returned ZERO!! Class: 0x"));
                            my_printf(packetBuf_st.cls_u8);
                            my_printf((" ID: 0x"));
                            my_printf(packetBuf_st.id_u8);
                        }
                    }
                    if (payloadAuto_pu8 != NULL) // Check if memory is already allocated - this should be impossible!
                    {
                        if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
                        {
                            my_printf("process: memory is already allocated for payloadAuto! Deleting...");
                        }
                        free(payloadAuto_pu8);
                        payloadAuto_pu8 = NULL; // Redundant?
                        packetAuto_st.payload_pu8 = payloadAuto_pu8;
                    }
                    payloadAuto_pu8 = malloc(maxPayload); // Allocate RAM for payloadAuto
                    packetAuto_st.payload_pu8 = payloadAuto_pu8;
                    if (payloadAuto_pu8 == NULL) // Check if the alloc failed
                    {
                        if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
                        {
                            my_printf("process: memory allocation failed for \"automatic\" message: Class: 0x%x ID: 0x%x",
                                    packetBuf_st.cls_u8, packetBuf_st.id_u8);
                            my_printf("process: \"automatic\" message could overwrite data");
                        }
                        // The RAM allocation failed so fall back to using incomingUBX (usually packetCfg) even though we risk overwriting data
                        gnss_pst->activePacketBuffer_e = UBLOX_PACKET_PACKETCFG;
                        incomingUBX_pst->cls_u8 = packetBuf_st.cls_u8; //Copy the class and ID into incomingUBX (usually packetCfg)
                        incomingUBX_pst->id_u8 = packetBuf_st.id_u8;
                        incomingUBX_pst->counter_u16 = packetBuf_st.counter_u16; //Copy over the .counter too
                    }
                    else
                    {
                        //The RAM allocation was successful so we start diverting data into packetAuto and process it
                        gnss_pst->activePacketBuffer_e = UBLOX_PACKET_PACKETAUTO;
                        packetAuto_st.cls_u8 = packetBuf_st.cls_u8; //Copy the class and ID into packetAuto
                        packetAuto_st.id_u8 = packetBuf_st.id_u8;
                        packetAuto_st.counter_u16 = packetBuf_st.counter_u16; //Copy over the .counter too
                        packetAuto_st.startingSpot_u16 = packetBuf_st.startingSpot_u16; //And the starting spot? (Probably redundant)
                        if (_printDebug == true)
                        {
                            my_printf("process: incoming \"automatic\" message: Class: 0x%x ID: 0x%x", packetBuf_st.cls_u8, packetBuf_st.id_u8);
                        }
                    }
                }
                else
                {
                    //This is not an ACK and we do not have a class and ID match
                    //so we should keep diverting data into packetBuf and ignore the payload
                    gnss_pst->ignoreThisPayload_b = true;
                }
            }
            else
            {
                // This is an ACK so it is to early to do anything with it
                // We need to wait until we have received the length and data bytes
                // So we should keep diverting data into packetBuf
            }
        }
        else if (gnss_pst->ubxFrameCounter_u16 == 4) //Length LSB
        {
            //We should save the length in packetBuf even if activePacketBuffer == UBLOX_PACKET_PACKETCFG
            packetBuf_st.len_u16 = incoming_u8; // (Duplication)
        }
        else if (gnss_pst->ubxFrameCounter_u16 == 5) //Length MSB
        {
            //We should save the length in packetBuf even if activePacketBuffer == UBLOX_PACKET_PACKETCFG
            packetBuf_st.len_u16 |= incoming_u8 << 8; // (Duplication)
        }
        else if (gnss_pst->ubxFrameCounter_u16 == 6) //This should be the first byte of the payload unless .len_u16 is zero
        {
            if (packetBuf_st.len_u16 == 0) // Check if length is zero (hopefully this is impossible!)
            {
                if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
                    my_printf("process: ZERO LENGTH packet received: Class: 0x%x ID: 0x",
                            packetBuf_st.cls_u8, packetBuf_st.id_u8);
                //If length is zero (!) this will be the first byte of the checksum so record it
                packetBuf_st.checksumA_u8 = incoming_u8;
            }
            else
            {
                //The length is not zero so record this byte in the payload
                packetBuf_st.payload_pu8[0] = incoming_u8;
            }
        }
        else if (gnss_pst->ubxFrameCounter_u16 == 7) //This should be the second byte of the payload unless .len_u16 is zero or one
        {
            if (packetBuf_st.len_u16 == 0) // Check if length is zero (hopefully this is impossible!)
            {
                //If length is zero (!) this will be the second byte of the checksum so record it
                packetBuf_st.checksumB_u8 = incoming_u8;
            }
            else if (packetBuf_st.len_u16 == 1) // Check if length is one
            {
                //The length is one so this is the first byte of the checksum
                packetBuf_st.checksumA_u8 = incoming_u8;
            }
            else // Length is >= 2 so this must be a payload byte
            {
                packetBuf_st.payload_pu8[1] = incoming_u8;
            }
            // Now that we have received two payload bytes, we can check for a matching ACK/NACK
            if ((gnss_pst->activePacketBuffer_e == UBLOX_PACKET_PACKETBUF) // If we are not already processing a data packet
                    && (packetBuf_st.cls_u8 == UBX_CLASS_ACK)                // and if this is an ACK/NACK
                    && (packetBuf_st.payload_pu8[0] == requestedClass_u8)        // and if the class matches
                    && (packetBuf_st.payload_pu8[1] == requestedID_u8))          // and if the ID matches
            {
                if (packetBuf_st.len_u16 == 2) // Check if .len_u16 is 2
                {
                    // Then this is a matching ACK so copy it into packetAck
                    gnss_pst->activePacketBuffer_e = UBLOX_PACKET_PACKETACK;
                    packetAck_st.cls_u8 = packetBuf_st.cls_u8;
                    packetAck_st.id_u8 = packetBuf_st.id_u8;
                    packetAck_st.len_u16 = packetBuf_st.len_u16;
                    packetAck_st.counter_u16 = packetBuf_st.counter_u16;
                    packetAck_st.payload_pu8[0] = packetBuf_st.payload_pu8[0];
                    packetAck_st.payload_pu8[1] = packetBuf_st.payload_pu8[1];
                }
                else // Length is not 2 (hopefully this is impossible!)
                {
                    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
                    {
                        my_printf("process: ACK received with .len_u16 != 2: Class: 0x%x ID: 0x%x len_u16: %d",
                                packetBuf_st.payload_pu8[0], packetBuf_st.payload_pu8[1], packetBuf_st.len_u16);
                    }
                }
            }
        }

        //Divert incoming into the correct buffer
        if (gnss_pst->activePacketBuffer_e == UBLOX_PACKET_PACKETACK)
        {
            Gnss_ProcessUBX(gnss_pst, incoming_u8, &packetAck_st, requestedClass_u8, requestedID_u8);
        }
        else if (gnss_pst->activePacketBuffer_e  == UBLOX_PACKET_PACKETCFG)
        {
            Gnss_ProcessUBX(gnss_pst, incoming_u8, incomingUBX_pst, requestedClass_u8, requestedID_u8);
        }
        else if (gnss_pst->activePacketBuffer_e == UBLOX_PACKET_PACKETBUF)
        {
            Gnss_ProcessUBX(gnss_pst, incoming_u8, &packetBuf_st, requestedClass_u8, requestedID_u8);
        }
        else // if (activePacketBuffer == UBLOX_PACKET_PACKETAUTO)
        {
            Gnss_ProcessUBX(gnss_pst, incoming_u8, &packetAuto_st, requestedClass_u8, requestedID_u8);
        }

        //Finally, increment the frame counter
        gnss_pst->ubxFrameCounter_u16++;
    }
    else if (gnss_pst->currentSentence_e == NMEA)
    {
//        processNMEA(incoming_u8); //Process each NMEA character
    }
    else if (gnss_pst->currentSentence_e == RTCM)
    {
//        processRTCMframe(incoming_u8); //Deal with RTCM bytes
    }
}

void Gnss_PrintPacket(struct Ublox_Gnss *gnss_pst, Ubx_Packet_tst *packet_st, bool alwaysPrintPayload_b)
{
    int index = 0;
    if (_printDebug == true)
    {
        char str[200];

        index = sprintf(str, "%s", "CLS:");
        if (packet_st->cls_u8 == UBX_CLASS_NAV) //1
        {
            index += sprintf(str+index, "%s", "NAV");
        }
        else if (packet_st->cls_u8 == UBX_CLASS_ACK) //5
        {
            index += sprintf(str+index, "%s", "ACK");
        }
        else if (packet_st->cls_u8 == UBX_CLASS_CFG) //6
        {
            index += sprintf(str+index, "%s", "CFG");
        }
        else if (packet_st->cls_u8 == UBX_CLASS_MON) //0x0A
        {
            index += sprintf(str+index, "%s", "MON");
        }
        else
        {
            index += sprintf(str+index, "%s%x", "0x", packet_st->cls_u8);
        }

        index += sprintf(str+index, "%s", " ID:");
        if (packet_st->cls_u8 == UBX_CLASS_NAV && packet_st->id_u8 == UBX_NAV_PVT)
        {
            index += sprintf(str+index,"%s", "PVT");
        }
        else if (packet_st->cls_u8 == UBX_CLASS_CFG && packet_st->id_u8 == UBX_CFG_RATE)
        {
            index += sprintf(str+index,"%s", "RATE");
        }
        else if (packet_st->cls_u8 == UBX_CLASS_CFG && packet_st->id_u8 == UBX_CFG_CFG)
        {
            index += sprintf(str+index,"%s", "SAVE");
        }
        else
        {
            index += sprintf(str+index, "%s%x", "0x", packet_st->id_u8);
        }

        index += sprintf(str+index, "%s%x", " Len: 0x", packet_st->len_u16);

        // Only print the payload is ignoreThisPayload is false otherwise
        // we could be printing gibberish from beyond the end of packetBuf
        if ((alwaysPrintPayload_b == true) || (gnss_pst->ignoreThisPayload_b == false))
        {
            index += sprintf(str+index, "%s", " Payload:");

            for (int x = 0; x < packet_st->len_u16; x++)
            {
                index += sprintf(str+index, "%s%02x ", "0x", packet_st->payload_pu8[x]);
            }
        }
        else
        {
            index += sprintf(str+index, "%s", " Payload: IGNORED");
        }

        my_printf("%s", str);
    }
}

void Gnss_SetSerialRate(struct Ublox_Gnss *gnss_pst, uint32_t baudrate_u32, uint8_t uartPort_u8, uint16_t maxTime_y16)
{
    if (Gnss_GetPortSettings(gnss_pst, uartPort_u8, maxTime_y16) == false)
    {
        my_printf("cannot set port setting.");
        return; //Something went wrong. Bail.
    }

//    if (_printDebug == true)
    {
        my_printf("Current baud rate: %d", ((uint32_t)payloadCfg_pu8[10] << 16) | ((uint32_t)payloadCfg_pu8[9] << 8) | payloadCfg_pu8[8]);
    }

    packetCfg_st.cls_u8 = UBX_CLASS_CFG;
    packetCfg_st.id_u8 = UBX_CFG_PRT;
    packetCfg_st.len_u16 = 20;
    packetCfg_st.startingSpot_u16 = 0;

    //payloadCfg is now loaded with current bytes. Change only the ones we need to
    payloadCfg_pu8[8] = baudrate_u32;
    payloadCfg_pu8[9] = baudrate_u32 >> 8;
    payloadCfg_pu8[10] = baudrate_u32 >> 16;
    payloadCfg_pu8[11] = baudrate_u32 >> 24;

//    if (_printDebug == true)
    {
        my_printf("New baud rate: %d", ((uint32_t)payloadCfg_pu8[10] << 16) | ((uint32_t)payloadCfg_pu8[9] << 8) | payloadCfg_pu8[8]);
    }

    ublox_status_ten retVal = Gnss_SendCmd(gnss_pst, &packetCfg_st, maxTime_y16, false);
    if (_printDebug == true)
    {
        my_printf("setSerialRate: sendCommand returned: %d", retVal);
    }
}

bool Gnss_CheckAutomatic(struct Ublox_Gnss *gnss_pst, uint8_t Class_u8, uint8_t ID_u8)
{
    bool result = false;
    switch (Class_u8)
    {
    case UBX_CLASS_NAV:
    {
        switch (ID_u8)
        {
        case UBX_NAV_PVT:
            if (packetUBXNAVPVT_pst != NULL) result = true;
            break;
        }
    }
    break;
    case UBX_CLASS_CFG:
    {
        switch (ID_u8)
        {
            case UBX_CFG_RATE:
                if (gnss_pst->packetUBXCFGRATE_pst != NULL) result = true;
                break;
        }
    }
    break;
    }
    return (result);
}

static uint32_t extractLong(Ubx_Packet_tst *msg_st, uint8_t spotToStart_u8)
{
  uint32_t val = 0;
  val |= (uint32_t)msg_st->payload_pu8[spotToStart_u8 + 0] << 8 * 0;
  val |= (uint32_t)msg_st->payload_pu8[spotToStart_u8 + 1] << 8 * 1;
  val |= (uint32_t)msg_st->payload_pu8[spotToStart_u8 + 2] << 8 * 2;
  val |= (uint32_t)msg_st->payload_pu8[spotToStart_u8 + 3] << 8 * 3;
  return (val);
}

static uint8_t extractByte(Ubx_Packet_tst *msg, uint8_t spotToStart)
{
  return (msg->payload_pu8[spotToStart]);
}

static uint16_t extractInt(Ubx_Packet_tst *msg_st, uint8_t spotToStart_u8)
{
  uint16_t val = 0;
  val |= (uint16_t)msg_st->payload_pu8[spotToStart_u8 + 0] << 8 * 0;
  val |= (uint16_t)msg_st->payload_pu8[spotToStart_u8 + 1] << 8 * 1;
  return (val);
}

static int32_t extractSignedLong(Ubx_Packet_tst *msg, uint8_t spotToStart)
{
  union // Use a union to convert from uint32_t to int32_t
  {
      uint32_t unsignedLong;
      int32_t signedLong;
  } unsignedSigned;

  unsignedSigned.unsignedLong = extractLong(msg, spotToStart);
  return (unsignedSigned.signedLong);
}

static int16_t extractSignedInt(Ubx_Packet_tst *msg, int8_t spotToStart)
{
  union // Use a union to convert from uint16_t to int16_t
  {
      uint16_t unsignedInt;
      int16_t signedInt;
  } stSignedInt;

  stSignedInt.unsignedInt = extractInt(msg, spotToStart);
  return (stSignedInt.signedInt);
}

void Gnss_ProcessUBXpacket(struct Ublox_Gnss *gnss_pst, Ubx_Packet_tst *msg_st)
{
    switch (msg_st->cls_u8)
    {
    case UBX_CLASS_NAV:
        if (msg_st->id_u8 == UBX_NAV_POSECEF && msg_st->len_u16 == UBX_NAV_POSECEF_LEN)
        {

        }
        else if (msg_st->id_u8 == UBX_NAV_STATUS && msg_st->len_u16 == UBX_NAV_STATUS_LEN)
        {

        }
        else if (msg_st->id_u8 == UBX_NAV_DOP && msg_st->len_u16 == UBX_NAV_DOP_LEN)
        {

        }
        else if (msg_st->id_u8 == UBX_NAV_ATT && msg_st->len_u16 == UBX_NAV_ATT_LEN)
        {

        }
        else if (msg_st->id_u8 == UBX_NAV_PVT && msg_st->len_u16 == UBX_NAV_PVT_LEN)
        {
          //Parse various byte fields into storage - but only if we have memory allocated for it
          if (packetUBXNAVPVT_pst != NULL)
          {
            packetUBXNAVPVT_pst->data.iTOW = extractLong(msg_st, 0);
            packetUBXNAVPVT_pst->data.year = extractInt(msg_st, 4);
            packetUBXNAVPVT_pst->data.month = extractByte(msg_st, 6);
            packetUBXNAVPVT_pst->data.day = extractByte(msg_st, 7);
            packetUBXNAVPVT_pst->data.hour = extractByte(msg_st, 8);
            packetUBXNAVPVT_pst->data.min = extractByte(msg_st, 9);
            packetUBXNAVPVT_pst->data.sec = extractByte(msg_st, 10);
            packetUBXNAVPVT_pst->data.valid.all = extractByte(msg_st, 11);
            packetUBXNAVPVT_pst->data.tAcc = extractLong(msg_st, 12);
            packetUBXNAVPVT_pst->data.nano = extractSignedLong(msg_st, 16); //Includes milliseconds
            packetUBXNAVPVT_pst->data.fixType = extractByte(msg_st, 20);
            packetUBXNAVPVT_pst->data.flags.all = extractByte(msg_st, 21);
            packetUBXNAVPVT_pst->data.flags2.all = extractByte(msg_st, 22);
            packetUBXNAVPVT_pst->data.numSV = extractByte(msg_st, 23);
            packetUBXNAVPVT_pst->data.lon = extractSignedLong(msg_st, 24);
            packetUBXNAVPVT_pst->data.lat = extractSignedLong(msg_st, 28);
            packetUBXNAVPVT_pst->data.height = extractSignedLong(msg_st, 32);
            packetUBXNAVPVT_pst->data.hMSL = extractSignedLong(msg_st, 36);
            packetUBXNAVPVT_pst->data.hAcc = extractLong(msg_st, 40);
            packetUBXNAVPVT_pst->data.vAcc = extractLong(msg_st, 44);
            packetUBXNAVPVT_pst->data.velN = extractSignedLong(msg_st, 48);
            packetUBXNAVPVT_pst->data.velE = extractSignedLong(msg_st, 52);
            packetUBXNAVPVT_pst->data.velD = extractSignedLong(msg_st, 56);
            packetUBXNAVPVT_pst->data.gSpeed = extractSignedLong(msg_st, 60);
            packetUBXNAVPVT_pst->data.headMot = extractSignedLong(msg_st, 64);
            packetUBXNAVPVT_pst->data.sAcc = extractLong(msg_st, 68);
            packetUBXNAVPVT_pst->data.headAcc = extractLong(msg_st, 72);
            packetUBXNAVPVT_pst->data.pDOP = extractInt(msg_st, 76);
            packetUBXNAVPVT_pst->data.flags3.all = extractByte(msg_st, 78);
            packetUBXNAVPVT_pst->data.headVeh = extractSignedLong(msg_st, 84);
            packetUBXNAVPVT_pst->data.magDec = extractSignedInt(msg_st, 88);
            packetUBXNAVPVT_pst->data.magAcc = extractInt(msg_st, 90);

            //Mark all datums as fresh (not read before)
            packetUBXNAVPVT_pst->moduleQueried.moduleQueried1.all = 0xFFFFFFFF;
            packetUBXNAVPVT_pst->moduleQueried.moduleQueried2.all = 0xFFFFFFFF;

            //Check if we need to copy the data for the callback
            if ((packetUBXNAVPVT_pst->callbackData != NULL) // If RAM has been allocated for the copy of the data
              && (packetUBXNAVPVT_pst->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
            {
              memcpy(&packetUBXNAVPVT_pst->callbackData->iTOW, &packetUBXNAVPVT_pst->data.iTOW, sizeof(UBX_NAV_PVT_data_t));
              packetUBXNAVPVT_pst->automaticFlags.flags.bits.callbackCopyValid = true;
            }

            //Check if we need to copy the data into the file buffer
            if (packetUBXNAVPVT_pst->automaticFlags.flags.bits.addToFileBuffer)
            {
//              storePacket(msg_st);
            }
          }
        }
        else if (msg_st->id_u8 == UBX_NAV_ODO && msg_st->len_u16 == UBX_NAV_ODO_LEN)
        {

        }
        else if (msg_st->id_u8 == UBX_NAV_VELECEF && msg_st->len_u16 == UBX_NAV_VELECEF_LEN)
        {

        }
        else if (msg_st->id_u8 == UBX_NAV_VELNED && msg_st->len_u16 == UBX_NAV_VELNED_LEN)
        {

        }
        else if (msg_st->id_u8 == UBX_NAV_HPPOSECEF && msg_st->len_u16 == UBX_NAV_HPPOSECEF_LEN)
        {

        }
        else if (msg_st->id_u8 == UBX_NAV_HPPOSLLH && msg_st->len_u16 == UBX_NAV_HPPOSLLH_LEN)
        {

        }
        else if (msg_st->id_u8 == UBX_NAV_CLOCK && msg_st->len_u16 == UBX_NAV_CLOCK_LEN)
        {

        }
        else if (msg_st->id_u8 == UBX_NAV_SVIN && msg_st->len_u16 == UBX_NAV_SVIN_LEN)
        {

        }
        else if (msg_st->id_u8 == UBX_NAV_RELPOSNED && ((msg_st->len_u16 == UBX_NAV_RELPOSNED_LEN) || (msg_st->len_u16 == UBX_NAV_RELPOSNED_LEN_F9)))
        {

        }
        break;

        case UBX_CLASS_CFG:
        if (msg_st->id_u8 == UBX_CFG_RATE && msg_st->len_u16 == UBX_CFG_RATE_LEN)
        {
          //Parse various byte fields into storage - but only if we have memory allocated for it
          if (gnss_pst->packetUBXCFGRATE_pst != NULL)
          {
              gnss_pst->packetUBXCFGRATE_pst->data.measRate = extractInt(msg_st, 0);
              gnss_pst->packetUBXCFGRATE_pst->data.navRate = extractInt( msg_st, 2);
              gnss_pst->packetUBXCFGRATE_pst->data.timeRef = extractInt( msg_st, 4);

            //Mark all datums as fresh (not read before)
              gnss_pst->packetUBXCFGRATE_pst->moduleQueried.moduleQueried.all = 0xFFFFFFFF;
          }
        }
        break;
    }
}

uint16_t Gnss_ExtractInt(Ubx_Packet_tst *msg_st, uint8_t spotToStart_u8)
{
    uint16_t val = 0;
    val |= (uint16_t)msg_st->payload_pu8[spotToStart_u8 + 0] << 8 * 0;
    val |= (uint16_t)msg_st->payload_pu8[spotToStart_u8 + 1] << 8 * 1;
    return (val);
}

void Gnss_AddToChecksum(struct Ublox_Gnss *gnss_pst, uint8_t incoming_u8)
{
    gnss_pst->rollingChecksumA_u8 += incoming_u8;
    gnss_pst->rollingChecksumB_u8 += gnss_pst->rollingChecksumA_u8;
}

bool Gnss_SetNavigationFrequency(struct Ublox_Gnss* gnss_pst, uint8_t navFreq_u8, uint16_t maxWait_u16)
{
    //Query the module
    packetCfg_st.cls_u8 = UBX_CLASS_CFG;
    packetCfg_st.id_u8 = UBX_CFG_RATE;
    packetCfg_st.len_u16 = 0;
    packetCfg_st.startingSpot_u16 = 0;

    //This will load the payloadCfg array with current settings of the given register
    if (Gnss_SendCmd(gnss_pst, &packetCfg_st, maxWait_u16, false) != UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
    {
        return (false);                                                       //If command send fails then bail
    }

    uint16_t measurementRate = 1000 / navFreq_u8;

    //payloadCfg is now loaded with current bytes. Change only the ones we need to
    payloadCfg_pu8[0] = measurementRate & 0xFF; //measRate LSB
    payloadCfg_pu8[1] = measurementRate >> 8;   //measRate MSB

    return ((Gnss_SendCmd(gnss_pst, &packetCfg_st, maxWait_u16, false)) == UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

bool Gnss_InitPacketUBXNAVPVT(struct Ublox_Gnss *gnss_pst)
{
    packetUBXNAVPVT_pst = (UBX_NAV_PVT_t *)malloc(sizeof(UBX_NAV_PVT_t)); //Allocate RAM for the main struct
    if (packetUBXNAVPVT_pst == NULL)
    {
      return (false);
    }
    packetUBXNAVPVT_pst->automaticFlags.flags.all = 0;
    packetUBXNAVPVT_pst->callbackPointer = NULL;
    packetUBXNAVPVT_pst->callbackData = NULL;
    packetUBXNAVPVT_pst->moduleQueried.moduleQueried1.all = 0;
    packetUBXNAVPVT_pst->moduleQueried.moduleQueried2.all = 0;
    return (true);
}

bool Gnss_InitModuleSWVersion()
{
    moduleSWVersion_pst = (moduleSWVersion_t *)malloc(sizeof(moduleSWVersion_pst)); //Allocate RAM for the main struct
    if (moduleSWVersion_pst == NULL)
    {
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        my_printf("initModuleSWVersion: PANIC! RAM allocation failed!");
      return (false);
    }
    moduleSWVersion_pst->versionHigh = 0;
    moduleSWVersion_pst->versionLow = 0;
    moduleSWVersion_pst->moduleQueried = false;
    return (true);
}

bool Gnss_InitPacketUBXESFSTATUS(struct Ublox_Gnss *gnss_pst)
{
    gnss_pst->packetUBXESFSTATUS_pst = (UBX_ESF_STATUS_t *)malloc(sizeof(UBX_ESF_STATUS_t)); //Allocate RAM for the main struct

    if (gnss_pst->packetUBXESFSTATUS_pst == NULL)
    {
        my_printf("initPacketUBXESFSTATUS: PANIC! RAM allocation failed!");
        return (false);
    }
    gnss_pst->packetUBXESFSTATUS_pst->automaticFlags.flags.all = 0;
    gnss_pst->packetUBXESFSTATUS_pst->callbackPointer = NULL;
    gnss_pst->packetUBXESFSTATUS_pst->callbackData = NULL;
    gnss_pst->packetUBXESFSTATUS_pst->moduleQueried.moduleQueried.all = 0;
    return (true);
}

bool Gnss_SetAutoPVT(struct Ublox_Gnss *gnss_pst, bool enabled_b, uint16_t maxWait_u16)
{
    return Gnss_SetAutoPVTrate(gnss_pst, enabled_b ? 1 : 0, true, maxWait_u16);
}

bool Gnss_SetAutoPVTImplicit(struct Ublox_Gnss *gnss_pst, bool enabled_b, bool implicitUpdate_b, uint16_t maxWait_u16)
{
    return Gnss_SetAutoPVTrate(gnss_pst, enabled_b ? 1 : 0, implicitUpdate_b, maxWait_u16);
}

bool Gnss_GetPVT(struct Ublox_Gnss *gnss_pst, uint16_t maxWait_u16)
{
    if (packetUBXNAVPVT_pst == NULL)
    {
        Gnss_InitPacketUBXNAVPVT(gnss_pst); //Check that RAM has been allocated for the PVT data
    }
    if (packetUBXNAVPVT_pst == NULL) //Bail if the RAM allocation failed
    {
        return (false);
    }

    if (packetUBXNAVPVT_pst->automaticFlags.flags.bits.automatic
            && packetUBXNAVPVT_pst->automaticFlags.flags.bits.implicitUpdate)
    {
        Gnss_CheckUbloxInternal(gnss_pst, &packetCfg_st, UBX_CLASS_NAV, UBX_NAV_PVT);
        return packetUBXNAVPVT_pst->moduleQueried.moduleQueried1.bits.all;
    }
    else if (packetUBXNAVPVT_pst->automaticFlags.flags.bits.automatic
            && !packetUBXNAVPVT_pst->automaticFlags.flags.bits.implicitUpdate)
    {
        return (false);
    }
    else
    {
      packetCfg_st.cls_u8 = UBX_CLASS_NAV;
      packetCfg_st.id_u8 = UBX_NAV_PVT;
      packetCfg_st.len_u16 = 0;
      packetCfg_st.startingSpot_u16 = 0;

      //The data is parsed as part of processing the response
      ublox_status_ten retVal = Gnss_SendCmd(gnss_pst, &packetCfg_st, maxWait_u16, false);

      if (retVal == UBLOX_STATUS_DATA_RECEIVED)
      {
          return (true);
      }

      if (retVal == UBLOX_STATUS_DATA_OVERWRITTEN)
      {
        return (true);
      }

      return (false);
    }
}

bool Gnss_SetAutoPVTcallback(struct Ublox_Gnss *gnss_pst, void (*callbackPointer)(UBX_NAV_PVT_data_t), uint16_t maxWait_u16)
{
    // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
    bool result = Gnss_SetAutoPVTImplicit(gnss_pst, true, false, maxWait_u16);
    if (!result)
      return (result); // Bail if setAutoPVT failed

    if (packetUBXNAVPVT_pst->callbackData == NULL) //Check if RAM has been allocated for the callback copy
    {
      packetUBXNAVPVT_pst->callbackData = (UBX_NAV_PVT_data_t *)malloc(sizeof(UBX_NAV_PVT_data_t)); //Allocate RAM for the main struct
    }

    if (packetUBXNAVPVT_pst->callbackData == NULL)
    {
        my_printf("setAutoPVTcallback: PANIC! RAM allocation failed!");
      return (false);
    }

    packetUBXNAVPVT_pst->callbackPointer = callbackPointer; // RAM has been allocated so now update the pointer

    return (true);
}

uint8_t Gnss_GetSIV(struct Ublox_Gnss *gnss_pst, uint16_t maxWait_u16)
{
    if (packetUBXNAVPVT_pst == NULL)
    {
        Gnss_InitPacketUBXNAVPVT(gnss_pst); //Check that RAM has been allocated for the PVT data
    }
    if (packetUBXNAVPVT_pst == NULL) //Bail if the RAM allocation failed
    {
        return 0;
    }

    if (packetUBXNAVPVT_pst->moduleQueried.moduleQueried1.bits.numSV == false)
    {
        Gnss_GetPVT(gnss_pst, maxWait_u16);
    }
    packetUBXNAVPVT_pst->moduleQueried.moduleQueried1.bits.numSV = false; //Since we are about to give this to user, mark this data as stale
    packetUBXNAVPVT_pst->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT_pst->data.numSV);
}

uint8_t Gnss_GetFixType(struct Ublox_Gnss *gnss_pst,uint16_t maxWait_u16)
{
    if (packetUBXNAVPVT_pst == NULL)
    {
        Gnss_InitPacketUBXNAVPVT(gnss_pst); //Check that RAM has been allocated for the PVT data
    }
    if (packetUBXNAVPVT_pst == NULL) //Bail if the RAM allocation failed
    {
        return 0;
    }

    if (packetUBXNAVPVT_pst->moduleQueried.moduleQueried1.bits.fixType == false)
    {
        Gnss_GetPVT(gnss_pst, maxWait_u16);
    }
    packetUBXNAVPVT_pst->moduleQueried.moduleQueried1.bits.fixType = false; //Since we are about to give this to user, mark this data as stale
    packetUBXNAVPVT_pst->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT_pst->data.fixType);
}

bool Gnss_GetGnssFixOk(struct Ublox_Gnss *gnss_pst,uint16_t maxWait_u16)
{
    if (packetUBXNAVPVT_pst == NULL)
    {
        Gnss_InitPacketUBXNAVPVT(gnss_pst); //Check that RAM has been allocated for the PVT data
    }
    if (packetUBXNAVPVT_pst == NULL) //Bail if the RAM allocation failed
    {
        return 0;
    }

    if (packetUBXNAVPVT_pst->moduleQueried.moduleQueried1.bits.gnssFixOK == false)
    {
        Gnss_GetPVT(gnss_pst, maxWait_u16);
    }
    packetUBXNAVPVT_pst->moduleQueried.moduleQueried1.bits.gnssFixOK = false; //Since we are about to give this to user, mark this data as stale
    packetUBXNAVPVT_pst->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT_pst->data.flags.bits.gnssFixOK);
}

int32_t Gnss_GetLongitude(struct Ublox_Gnss *gnss_pst,uint16_t maxWait_u16)
{
    if (packetUBXNAVPVT_pst == NULL)
    {
        Gnss_InitPacketUBXNAVPVT(gnss_pst); //Check that RAM has been allocated for the PVT data
    }
    if (packetUBXNAVPVT_pst == NULL) //Bail if the RAM allocation failed
    {
        return 0;
    }

    if (packetUBXNAVPVT_pst->moduleQueried.moduleQueried1.bits.lon == false)
    {
        Gnss_GetPVT(gnss_pst, maxWait_u16);
    }
    packetUBXNAVPVT_pst->moduleQueried.moduleQueried1.bits.lon = false; //Since we are about to give this to user, mark this data as stale
    packetUBXNAVPVT_pst->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT_pst->data.lon);
}
int32_t Gnss_GetLatitude(struct Ublox_Gnss *gnss_pst,uint16_t maxWait_u16)
{
    if (packetUBXNAVPVT_pst == NULL)
    {
        Gnss_InitPacketUBXNAVPVT(gnss_pst); //Check that RAM has been allocated for the PVT data
    }
    if (packetUBXNAVPVT_pst == NULL) //Bail if the RAM allocation failed
    {
        return 0;
    }

    if (packetUBXNAVPVT_pst->moduleQueried.moduleQueried1.bits.lat == false)
    {
        Gnss_GetPVT(gnss_pst, maxWait_u16);
    }
    packetUBXNAVPVT_pst->moduleQueried.moduleQueried1.bits.lat = false; //Since we are about to give this to user, mark this data as stale
    packetUBXNAVPVT_pst->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT_pst->data.lat);
}

int32_t Gnss_GetAltitude(struct Ublox_Gnss *gnss_pst,uint16_t maxWait_u16)
{
    if (packetUBXNAVPVT_pst == NULL)
    {
        Gnss_InitPacketUBXNAVPVT(gnss_pst); //Check that RAM has been allocated for the PVT data
    }
    if (packetUBXNAVPVT_pst == NULL) //Bail if the RAM allocation failed
    {
        return 0;
    }

    if (packetUBXNAVPVT_pst->moduleQueried.moduleQueried1.bits.hMSL == false)
    {
        Gnss_GetPVT(gnss_pst, maxWait_u16);
    }
    packetUBXNAVPVT_pst->moduleQueried.moduleQueried1.bits.hMSL = false; //Since we are about to give this to user, mark this data as stale
    packetUBXNAVPVT_pst->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT_pst->data.hMSL);
}

//Get the current altitude in mm according to mean sea level
//Ellipsoid model: https://www.esri.com/news/arcuser/0703/geoid1of3.html
//Difference between Ellipsoid Model and Mean Sea Level: https://eos-gnss.com/elevation-for-beginners/
int32_t Gnss_GetAltitudeMSL(struct Ublox_Gnss *gnss_pst,uint16_t maxWait_u16)
{
    if (packetUBXNAVPVT_pst == NULL)
    {
        Gnss_InitPacketUBXNAVPVT(gnss_pst); //Check that RAM has been allocated for the PVT data
    }
    if (packetUBXNAVPVT_pst == NULL) //Bail if the RAM allocation failed
    {
        return 0;
    }

    if (packetUBXNAVPVT_pst->moduleQueried.moduleQueried1.bits.hMSL == false)
    {
        Gnss_GetPVT(gnss_pst, maxWait_u16);
    }
    packetUBXNAVPVT_pst->moduleQueried.moduleQueried1.bits.hMSL = false; //Since we are about to give this to user, mark this data as stale
    packetUBXNAVPVT_pst->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT_pst->data.hMSL);
}

int32_t Gnss_GetGroundSpeed(struct Ublox_Gnss *gnss_pst,uint16_t maxWait_u16)
{
    if (packetUBXNAVPVT_pst == NULL)
    {
        Gnss_InitPacketUBXNAVPVT(gnss_pst); //Check that RAM has been allocated for the PVT data
    }
    if (packetUBXNAVPVT_pst == NULL) //Bail if the RAM allocation failed
    {
        return 0;
    }

    if (packetUBXNAVPVT_pst->moduleQueried.moduleQueried2.bits.gSpeed == false)
    {
        Gnss_GetPVT(gnss_pst, maxWait_u16);
    }
    packetUBXNAVPVT_pst->moduleQueried.moduleQueried2.bits.gSpeed = false; //Since we are about to give this to user, mark this data as stale
    packetUBXNAVPVT_pst->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT_pst->data.gSpeed);
}

uint32_t Gnss_GetSpeedAccEst(struct Ublox_Gnss *gnss_pst,uint16_t maxWait_u16)
{
    if (packetUBXNAVPVT_pst == NULL)
    {
        Gnss_InitPacketUBXNAVPVT(gnss_pst); //Check that RAM has been allocated for the PVT data
    }
    if (packetUBXNAVPVT_pst == NULL) //Bail if the RAM allocation failed
    {
        return 0;
    }

    if (packetUBXNAVPVT_pst->moduleQueried.moduleQueried2.bits.sAcc == false)
    {
        Gnss_GetPVT(gnss_pst, maxWait_u16);
    }
    packetUBXNAVPVT_pst->moduleQueried.moduleQueried2.bits.sAcc = false; //Since we are about to give this to user, mark this data as stale
    packetUBXNAVPVT_pst->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT_pst->data.sAcc);
}

bool Gnss_SetAutoPVTrate(struct Ublox_Gnss *gnss_pst, uint8_t rate_u8, bool implicitUpdate_b, uint16_t maxWait_u16)
{
    if (packetUBXNAVPVT_pst == NULL)
    {
        Gnss_InitPacketUBXNAVPVT(gnss_pst); //Check that RAM has been allocated for the PVT data
    }
    if (packetUBXNAVPVT_pst == NULL) //Only attempt this if RAM allocation was successful
    {
        return false;
    }

    if (rate_u8 > 127)
    {
        rate_u8 = 127;
    }

    packetCfg_st.cls_u8 = UBX_CLASS_CFG;
    packetCfg_st.id_u8 = UBX_CFG_MSG;
    packetCfg_st.len_u16 = 3;
    packetCfg_st.startingSpot_u16 = 0;
    payloadCfg_pu8[0] = UBX_CLASS_NAV;
    payloadCfg_pu8[1] = UBX_NAV_PVT;
    payloadCfg_pu8[2] = rate_u8; // rate relative to navigation freq.

    bool ok = ((Gnss_SendCmd(gnss_pst, &packetCfg_st, maxWait_u16, false)) == UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
    if (ok)
    {
        packetUBXNAVPVT_pst->automaticFlags.flags.bits.automatic = (rate_u8 > 0);
        packetUBXNAVPVT_pst->automaticFlags.flags.bits.implicitUpdate = implicitUpdate_b;
    }
    packetUBXNAVPVT_pst->moduleQueried.moduleQueried1.bits.all = false;

    return ok;
}

uint16_t Gnss_GetMaxPayloadSize(struct Ublox_Gnss *gnss_pst, uint8_t Class_u8, uint8_t ID_u8)
{
    uint16_t maxSize = 0;
    switch (Class_u8)
    {
      case UBX_CLASS_NAV:
      {
        switch (ID_u8)
        {
          case UBX_NAV_POSECEF:
            maxSize = UBX_NAV_POSECEF_LEN;
          break;
          case UBX_NAV_STATUS:
            maxSize = UBX_NAV_STATUS_LEN;
          break;
          case UBX_NAV_DOP:
            maxSize = UBX_NAV_DOP_LEN;
          break;
          case UBX_NAV_ATT:
            maxSize = UBX_NAV_ATT_LEN;
          break;
          case UBX_NAV_PVT:
            maxSize = UBX_NAV_PVT_LEN;
          break;
          case UBX_NAV_ODO:
            maxSize = UBX_NAV_ODO_LEN;
          break;
          case UBX_NAV_VELECEF:
            maxSize = UBX_NAV_VELECEF_LEN;
          break;
          case UBX_NAV_VELNED:
            maxSize = UBX_NAV_VELNED_LEN;
          break;
          case UBX_NAV_HPPOSECEF:
            maxSize = UBX_NAV_HPPOSECEF_LEN;
          break;
          case UBX_NAV_HPPOSLLH:
            maxSize = UBX_NAV_HPPOSLLH_LEN;
          break;
          case UBX_NAV_CLOCK:
            maxSize = UBX_NAV_CLOCK_LEN;
          break;
          case UBX_NAV_SVIN:
            maxSize = UBX_NAV_SVIN_LEN;
          break;
          case UBX_NAV_RELPOSNED:
            maxSize = UBX_NAV_RELPOSNED_LEN_F9;
          break;
        }
      }
      break;

      case UBX_CLASS_CFG:
      {
        switch (ID_u8)
        {
          case UBX_CFG_RATE:
            maxSize = UBX_CFG_RATE_LEN;
          break;
        }
      }
      break;
    }
    return (maxSize);
}

bool Gnss_GetEsfInfo(struct Ublox_Gnss *gnss_pst, uint16_t maxWait_u16)
{
    // ***** ESF STATUS automatic support
    return (Gnss_GetESFSTATUS(gnss_pst, maxWait_u16));
}

bool Gnss_GetESFSTATUS(struct Ublox_Gnss *gnss_pst, uint16_t maxWait_u16)
{
    if (gnss_pst->packetUBXESFSTATUS_pst == NULL)
    {
        Gnss_InitPacketUBXESFSTATUS(gnss_pst); //Check that RAM has been allocated for the ESF status data
    }
    if (gnss_pst->packetUBXESFSTATUS_pst == NULL) //Only attempt this if RAM allocation was successful
    {
      return false;
    }

    if (gnss_pst->packetUBXESFSTATUS_pst->automaticFlags.flags.bits.automatic
            && gnss_pst->packetUBXESFSTATUS_pst->automaticFlags.flags.bits.implicitUpdate)
    {
      //The GPS is automatically reporting, we just check whether we got unread data
      Gnss_CheckUbloxInternal(gnss_pst, &packetCfg_st, UBX_CLASS_ESF, UBX_ESF_STATUS);
      return gnss_pst->packetUBXESFSTATUS_pst->moduleQueried.moduleQueried.bits.all;
    }
    else if (gnss_pst->packetUBXESFSTATUS_pst->automaticFlags.flags.bits.automatic && !gnss_pst->packetUBXESFSTATUS_pst->automaticFlags.flags.bits.implicitUpdate)
    {
      return (false);
    }
    else
    {
      //The GPS is not automatically reporting HNR PVT so we have to poll explicitly
      packetCfg_st.cls_u8 = UBX_CLASS_ESF;
      packetCfg_st.id_u8 = UBX_ESF_STATUS;
      packetCfg_st.len_u16 = 0;
      packetCfg_st.startingSpot_u16 = 0;

      //The data is parsed as part of processing the response
      ublox_status_ten retVal = Gnss_SendCmd(gnss_pst, &packetCfg_st, maxWait_u16, false);

      if (retVal == UBLOX_STATUS_DATA_RECEIVED)
      {
        return (true);
      }

      if (retVal == UBLOX_STATUS_DATA_OVERWRITTEN)
      {
        return (true);
      }
      return (false);
    }

    return (false); // Trap. We should never get here...
}

uint8_t Gnss_GetProtocolVersionHigh(struct Ublox_Gnss *gnss_pst,uint16_t maxWait)
{
    if (moduleSWVersion_pst == NULL) {
        Gnss_InitModuleSWVersion(); //Check that RAM has been allocated for the SW version
    }
    if (moduleSWVersion_pst == NULL) //Bail if the RAM allocation failed
    {
      return (false);
    }

    if (moduleSWVersion_pst->moduleQueried == false)
    {
      Gnss_GetProtocolVersion(gnss_pst, maxWait);
    }
    return (moduleSWVersion_pst->versionHigh);
}

uint8_t Gnss_GetProtocolVersionLow(struct Ublox_Gnss *gnss_pst,uint16_t maxWait)
{
    if (moduleSWVersion_pst == NULL) Gnss_InitModuleSWVersion(); //Check that RAM has been allocated for the SW version
    if (moduleSWVersion_pst == NULL) //Bail if the RAM allocation failed
      return (false);

    if (moduleSWVersion_pst->moduleQueried == false)
        Gnss_GetProtocolVersion(gnss_pst, maxWait);
    return (moduleSWVersion_pst->versionLow);
}

//Get the current protocol version of the u-blox module we're communicating with
//This is helpful when deciding if we should call the high-precision Lat/Long (HPPOSLLH) or the regular (POSLLH)
bool Gnss_GetProtocolVersion(struct Ublox_Gnss *gnss_pst,uint16_t maxWait)
{
    if (moduleSWVersion_pst == NULL)
    {
        Gnss_InitModuleSWVersion(); //Check that RAM has been allocated for the SW version
    }
    if (moduleSWVersion_pst == NULL) //Bail if the RAM allocation failed
    {
      return (false);
    }

    //Send packet with only CLS and ID, length of zero. This will cause the module to respond with the contents of that CLS/ID.
    packetCfg_st.cls_u8 = UBX_CLASS_MON;
    packetCfg_st.id_u8 = UBX_MON_VER;

    packetCfg_st.len_u16 = 0;
    packetCfg_st.startingSpot_u16 = 40; //Start at first "extended software information" string

    if (Gnss_SendCmd(gnss_pst, &packetCfg_st, maxWait, false) != UBLOX_STATUS_DATA_RECEIVED) // We are only expecting data (no ACK)
    {
      return (false);
    }

    //We will step through the payload looking at each extension field of 30 bytes
    for (uint8_t extensionNumber = 0; extensionNumber < 10; extensionNumber++)
    {
      //Now we need to find "PROTVER=18.00" in the incoming byte stream
      if ((payloadCfg_pu8[(30 * extensionNumber) + 0] == 'P') && (payloadCfg_pu8[(30 * extensionNumber) + 6] == 'R'))
      {
        moduleSWVersion_pst->versionHigh = (payloadCfg_pu8[(30 * extensionNumber) + 8] - '0') * 10 + (payloadCfg_pu8[(30 * extensionNumber) + 9] - '0');  //Convert '18' to 18
        moduleSWVersion_pst->versionLow = (payloadCfg_pu8[(30 * extensionNumber) + 11] - '0') * 10 + (payloadCfg_pu8[(30 * extensionNumber) + 12] - '0'); //Convert '00' to 00
        moduleSWVersion_pst->moduleQueried = true; // Mark this data as new

        if (_printDebug == true)
        {
          my_printf("Protocol version: 02x.02x",moduleSWVersion_pst->versionHigh,moduleSWVersion_pst->versionLow);
        }
        return (true); //Success!
      }
    }

    return (false); //We failed
}
