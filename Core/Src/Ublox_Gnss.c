/* *****************************************************************************
 *
 *
 ******************************************************************************/

/* *****************************************************************************
 * Files inclusion
 ******************************************************************************/
#include "Ublox_Gnss.h"

/* *****************************************************************************
 * Internal Macros
 ******************************************************************************/


/* *****************************************************************************
 * Static function declaration
 ******************************************************************************/
static bool _printDebug = true;
static bool _printLimitedDebug = true;
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

uint8_t testBuff[256] = {0};

uint8_t g_incomming = 0x00;

static int count = 0;
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

        if(gnss_pst->packetCfgPayloadSize == 0)
        {
            gnss_pst->setPacketCfgPayloadSize(gnss_pst, MAX_PAYLOAD_SIZE);
        }

//        connected = gnss_pst->isConnected(gnss_pst);
//        if (!connected)
//        {
//            connected = gnss_pst->isConnected(gnss_pst);
//        }
//
//        if (!connected)
//        {
//            connected = gnss_pst->isConnected(gnss_pst);
//        }
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
    if (gnss_pst->sendCommand(gnss_pst, &packetCfg_st, maxWait_u16, false) != UBLOX_STATUS_DATA_RECEIVED)
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
    return ((gnss_pst->sendCommand(gnss_pst, &packetCfg_st, maxWait_u16, false)) == UBLOX_STATUS_DATA_SENT);
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

    gnss_pst->calcChecksum(gnss_pst, outgoingUBX_st); //Sets checksum A and B bytes of the packet

    if (_printDebug != true)
    {
      my_printf("\nSending: ");
      gnss_pst->printPacket(gnss_pst, outgoingUBX_st, true); // Always print payload
    }

    if (gnss_pst->commTypes_en == COMM_TYPE_SERIAL)
    {
        gnss_pst->sendSerialCommand(gnss_pst, outgoingUBX_st);
        /* TODO: should delay so that transmission is finished */
        HAL_Delay(250);
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
        retVal = gnss_pst->waitForACKResponse(gnss_pst, outgoingUBX_st, outgoingUBX_st->cls_u8, outgoingUBX_st->id_u8, maxWait_u16);
      }
      else
      {
        if (_printDebug == true)
        {
          my_printf("sendCommand: Waiting for No ACK response");
        }
        retVal = gnss_pst->waitForNoACKResponse(gnss_pst, outgoingUBX_st, outgoingUBX_st->cls_u8, outgoingUBX_st->id_u8, maxWait_u16); //Wait for Ack response
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

        gnss_pst->serial_pst->write(gnss_pst->serial_pst, pkg_pu8, outgoingUBX_st->len_u16 + 8u);

        free(pkg_pu8);
    }
    //Write header bytes
    //μ - oh ublox, you're funny. I will call you micro-blox from now on.
//    HAL_UART_Transmit_DMA(gnss_pst->uartHdl_pst, (UBX_SYNCH_1), sizeof(UBX_SYNCH_1)/sizeof(uint8_t));
//    HAL_UART_Transmit_DMA(gnss_pst->uartHdl_pst, (UBX_SYNCH_2), sizeof(UBX_SYNCH_2)/sizeof(uint8_t)); //b
//    HAL_UART_Transmit_DMA(gnss_pst->uartHdl_pst, outgoingUBX_st->cls_u8, sizeof(outgoingUBX_st->cls_u8)/sizeof(uint8_t));
//    HAL_UART_Transmit_DMA(gnss_pst->uartHdl_pst, outgoingUBX_st->id_u8, sizeof(outgoingUBX_st->id_u8)/sizeof(uint8_t));
//    HAL_UART_Transmit_DMA(gnss_pst->uartHdl_pst, outgoingUBX_st->len_u16 & 0xFF, sizeof(uint8_t)); //LSB
//    HAL_UART_Transmit_DMA(gnss_pst->uartHdl_pst, outgoingUBX_st->len_u16 >> 8, sizeof(uint8_t));   //MSB

    //Write payload.
//    for (int i = 0; i < outgoingUBX_st->len_u16; i++)
//    {
//        HAL_UART_Transmit_DMA(gnss_pst->uartHdl_pst, outgoingUBX_st->payload_pu8[i], sizeof(uint8_t));
//    }

    //Write checksum
//    HAL_UART_Transmit_DMA(gnss_pst->uartHdl_pst, outgoingUBX_st->checksumA_u8, sizeof(uint8_t));
//    HAL_UART_Transmit_DMA(gnss_pst->uartHdl_pst, outgoingUBX_st->checksumB_u8, sizeof(uint8_t));
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

    unsigned long startTime = HAL_GetTick();
    unsigned long current;
    uint8_t data_u8;

    my_printf("start=%d, maxTime=%d", startTime, maxTime_u16);
//    while (HAL_GetTick() - startTime < maxTime_u16)
    do
    {
        if(gnss_pst->serial_pst->available(gnss_pst->serial_pst))
        {
            gnss_pst->serial_pst->read(gnss_pst->serial_pst, &data_u8);
            gnss_pst->process(gnss_pst, data_u8, outgoingUBX_pst, requestedClass_u8, requestedID_u8);
        }

        current = HAL_GetTick();
        if(current - startTime >= 20)
        {
            my_printf("startTimeadadsfadfadfadfadfadfadfad=%d, cu%d", startTime, current);
        }
        HAL_Delay(1);
    } while (current - startTime < maxTime_u16);

    my_printf("curr=%d", current);

    // We have timed out...
    // If the outgoingUBX_pst->classAndIDmatch_en is VALID then we can take a gamble and return DATA_RECEIVED
    // even though we did not get an ACK
    if ((outgoingUBX_pst->classAndIDmatch_en == UBLOX_PACKET_VALIDITY_VALID) && (packetAck_st.classAndIDmatch_en == UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (outgoingUBX_pst->valid_en == UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX_pst->cls_u8 == requestedClass_u8) && (outgoingUBX_pst->id_u8 == requestedID_u8))
    {
      if (_printDebug == true)
      {
        my_printf(("waitForACKResponse: TIMEOUT with valid_en data after "));
//        my_printf(millis() - startTime);
//        my_printf(F(" msec. "));
      }
      return (UBLOX_STATUS_DATA_RECEIVED); //We received valid_en data... But no ACK!
    }

    if (_printDebug == true)
    {
      my_printf(("waitForACKResponse: TIMEOUT after "));
//      my_printf(millis() - startTime);
//      my_printf(F(" msec."));
    }

    my_printf("exit");
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
      if (gnss_pst->checkUbloxInternal(gnss_pst, outgoingUBX, requestedClass, requestedID) == true) //See if new data is available. Process bytes as they come in.
      {

        // If outgoingUBX->classAndIDmatch_en is VALID
        // and outgoingUBX->valid_en is _still_ VALID and the class and ID _still_ match
        // then we can be confident that the data in outgoingUBX is valid_en
        if ((outgoingUBX->classAndIDmatch_en == UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->valid_en == UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls_u8 == requestedClass) && (outgoingUBX->id_u8 == requestedID))
        {
          if (_printDebug == true)
          {
            my_printf(("waitForNoACKResponse: valid_en data with CLS/ID match after "));
//            my_printf(millis() - startTime);
//            my_printf(F(" msec"));
          }
          return (UBLOX_STATUS_DATA_RECEIVED); //We received valid_en data!
        }

        // If the outgoingUBX->classAndIDmatch_en is VALID
        // but the outgoingUBX->cls_u8 or ID no longer match then we can be confident that we had
        // valid_en data but it has been or is currently being overwritten by another packet (e.g. PVT).
        // If (e.g.) a PVT packet is _being_ received: outgoingUBX->valid_en will be NOT_DEFINED
        // If (e.g.) a PVT packet _has been_ received: outgoingUBX->valid_en will be VALID (or just possibly NOT_VALID)
        // So we cannot use outgoingUBX->valid_en as part of this check.
        // Note: the addition of packetBuf_st should make this check redundant!
        else if ((outgoingUBX->classAndIDmatch_en == UBLOX_PACKET_VALIDITY_VALID) && ((outgoingUBX->cls_u8 != requestedClass) || (outgoingUBX->id_u8 != requestedID)))
        {
          if (_printDebug == true)
          {
            my_printf(("waitForNoACKResponse: data being OVERWRITTEN after "));
//            my_printf(millis() - startTime);
//            my_printf(F(" msec"));
          }
          return (UBLOX_STATUS_DATA_OVERWRITTEN); // Data was valid_en but has been or is being overwritten
        }

        // If outgoingUBX->classAndIDmatch_en is NOT_DEFINED
        // and outgoingUBX->valid_en is VALID then this must be (e.g.) a PVT packet
        else if ((outgoingUBX->classAndIDmatch_en == UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (outgoingUBX->valid_en == UBLOX_PACKET_VALIDITY_VALID))
        {
          // if (_printDebug == true)
          // {
          //   my_printf(("waitForNoACKResponse: valid_en but UNWANTED data after "));
          //   my_printf(millis() - startTime);
          //   my_printf((" msec. Class: "));
          //   my_printf(outgoingUBX->cls_u8);
          //   my_printf((" ID: "));
          //   my_printf(outgoingUBX->id_u8);
          // }
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

      delayMicroseconds(500);
    }
    if (_printDebug == true)
    {
      my_printf(("waitForNoACKResponse: TIMEOUT after "));
//      my_printf(millis() - startTime);
//      my_printf(F(" msec. No packet received."));
    }
    return (UBLOX_STATUS_TIMEOUT);

}

bool Gnss_EnableNMEAMessage(struct Ublox_Gnss *gnss_pst, uint8_t msgID_u8, uint8_t portID_u8, uint8_t sendRate_u8, uint16_t maxWait_u16)
{
    return (gnss_pst->configureMsg_pf(gnss_pst, UBX_CLASS_NMEA, msgID_u8, portID_u8, sendRate_u8, maxWait_u16));
}

bool Gnss_DisableNMEAMessage(struct Ublox_Gnss *gnss_pst, uint8_t msgID_u8, uint8_t portID_u8, uint16_t maxWait_u16)
{
    return (gnss_pst->enableNMEAMessage(gnss_pst, msgID_u8, portID_u8, 0, maxWait_u16));
}

bool Gnss_IsConnected(struct Ublox_Gnss *gnss_pst)
{
    return gnss_pst->isConnectedT(gnss_pst, 1100);
}

bool Gnss_IsConnectedT(struct Ublox_Gnss *gnss_pst, uint16_t maxWait_u16)
{
    return gnss_pst->getNavigationFrequencyInternal(gnss_pst, maxWait_u16);
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
        gnss_pst->initPacketUBXCFGRATE(gnss_pst);
        if (gnss_pst->packetUBXCFGRATE_pst == NULL)
        {
            return false;
        }
    }

    if (gnss_pst->packetUBXCFGRATE_pst->automaticFlags.flags.bits.automatic && gnss_pst->packetUBXCFGRATE_pst->automaticFlags.flags.bits.implicitUpdate)
    {
      //The GPS is automatically reporting, we just check whether we got unread data
        gnss_pst->checkUbloxInternal(gnss_pst, &packetCfg_st, UBX_CLASS_CFG, UBX_CFG_RATE);
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
      ublox_status_ten retVal = gnss_pst->sendCommand(gnss_pst, &packetCfg_st, maxWait_u16, false);

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

bool Gnss_CheckUbloxInternal(struct Ublox_Gnss *gnss_pst, Ubx_Packet_tst *incomingUBX_pst, uint8_t requestedClass_u8, uint8_t requestedID_u8)
{
    if (gnss_pst->commTypes_en == COMM_TYPE_SERIAL)
    {
        return (gnss_pst->checkUbloxSerial(gnss_pst, incomingUBX_pst, requestedClass_u8, requestedID_u8));
    }
    return false;
}

bool Gnss_CheckUbloxSerial(struct Ublox_Gnss *gnss_pst, Ubx_Packet_tst *incomingUBX_pst, uint8_t requestedClass_u8, uint8_t requestedID_u8)
{
    uint8_t data_u8;

    while( gnss_pst->serial_pst->available(gnss_pst->serial_pst) )
    {
        gnss_pst->serial_pst->read(gnss_pst->serial_pst, &data_u8);

        gnss_pst->process(gnss_pst, data_u8, incomingUBX_pst, requestedClass_u8, requestedID_u8);
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

    return ((gnss_pst->sendCommand(gnss_pst, &packetCfg_st, maxWait_u16, false)) == UBLOX_STATUS_DATA_RECEIVED); // We are expecting data and an ACK
}

bool Gnss_SetPortOutput(struct Ublox_Gnss *gnss_pst, uint8_t portID, uint8_t outStreamSettings, uint16_t maxWait)
{
    //Get the current config values for this port ID
//     if (gnss_pst->getPortSettings(gnss_pst, portID, maxWait) == false)
//     {
//       return (false);
//     }

     packetCfg_st.cls_u8 = UBX_CLASS_CFG;
     packetCfg_st.id_u8 = UBX_CFG_PRT;
     packetCfg_st.len_u16 = 20;
     packetCfg_st.startingSpot_u16 = 0;

     //payloadCfg is now loaded with current bytes. Change only the ones we need to
     payloadCfg_pu8[14] = outStreamSettings; //OutProtocolMask LSB - Set outStream bits

     return ((gnss_pst->sendCommand(gnss_pst, &packetCfg_st, maxWait, false)) == UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

bool Gnss_SetUART1Output(struct Ublox_Gnss *gnss_pst, uint8_t comSettings, uint16_t maxWait)
{
    return (gnss_pst->setPortOutput(gnss_pst, COM_PORT_UART1, comSettings, maxWait));
}

bool Gnss_SetUART2Output(struct Ublox_Gnss *gnss_pst, uint8_t comSettings, uint16_t maxWait)
{
    return (gnss_pst->setPortOutput(gnss_pst, COM_PORT_UART2, comSettings, maxWait));
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
       maximum_payload_size = gnss_pst->getMaxPayloadSize(gnss_pst, incomingUBX_pst->cls_u8, incomingUBX_pst->id_u8);
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
   //Stop at len+4 as this is the checksum bytes to that should not be added to the rolling checksum
   if (incomingUBX_pst->counter_u16 < incomingUBX_pst->len_u16 + 4)
   {
     gnss_pst->addToChecksum(gnss_pst, incoming_u8);
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
       else if (gnss_pst->checkAutomatic(gnss_pst, incomingUBX_pst->cls_u8, incomingUBX_pst->id_u8))
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

           gnss_pst->printPacket(gnss_pst, incomingUBX_pst, false);

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
           gnss_pst->processUBXpacket(gnss_pst, incomingUBX_pst);
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
       free(payloadAuto_pu8);
       payloadAuto_pu8 = NULL; // Redundant?
       packetAuto_st.payload_pu8 = payloadAuto_pu8;
     }
   }
   else //Load this byte into the payload array
   {
     //If an automatic packet comes in asynchronously, we need to fudge the startingSpot
     uint16_t startingSpot = incomingUBX_pst->startingSpot_u16;
     if (gnss_pst->checkAutomatic(gnss_pst, incomingUBX_pst->cls_u8, incomingUBX_pst->id_u8))
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

   // incomingUBX->counter should never reach maximum_payload_size + class + id + len[2] + checksum[2]
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

   //Increment the counter
   incomingUBX_pst->counter_u16++;

   }
}

void Gnss_Process(struct Ublox_Gnss *gnss_pst, uint8_t incoming_u8, Ubx_Packet_tst *incomingUBX_pst, uint8_t requestedClass_u8, uint8_t requestedID_u8)
{
    testBuff[count] = incoming_u8;
    count++;
//    if(gnss_pst->currentSentence_e == UBX)
//        my_printf("incom 0x%02x", incoming_u8);
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
                else if (gnss_pst->checkAutomatic(gnss_pst, packetBuf_st.cls_u8, packetBuf_st.id_u8))
                {
                    //This is not the message we were expecting but it has its own storage and so we should process it anyway.
                    //We'll try to use packetAuto to buffer the message (so it can't overwrite anything in packetCfg).
                    //We need to allocate memory for the packetAuto payload (payloadAuto) - and delete it once
                    //reception is complete.
                    uint16_t maxPayload = gnss_pst->getMaxPayloadSize(gnss_pst, packetBuf_st.cls_u8, packetBuf_st.id_u8); // Calculate how much RAM we need
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
        else if (gnss_pst->ubxFrameCounter_u16 == 6) //This should be the first byte of the payload unless .len is zero
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
        else if (gnss_pst->ubxFrameCounter_u16 == 7) //This should be the second byte of the payload unless .len is zero or one
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
                if (packetBuf_st.len_u16 == 2) // Check if .len is 2
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
                        my_printf("process: ACK received with .len != 2: Class: 0x%x ID: 0x%x len: %d",
                                packetBuf_st.payload_pu8[0], packetBuf_st.payload_pu8[1], packetBuf_st.len_u16);
                    }
                }
            }
        }

        //Divert incoming into the correct buffer
        if (gnss_pst->activePacketBuffer_e == UBLOX_PACKET_PACKETACK)
        {
            gnss_pst->processUBX(gnss_pst, incoming_u8, &packetAck_st, requestedClass_u8, requestedID_u8);
        }
        else if (gnss_pst->activePacketBuffer_e  == UBLOX_PACKET_PACKETCFG)
        {
            gnss_pst->processUBX(gnss_pst, incoming_u8, incomingUBX_pst, requestedClass_u8, requestedID_u8);
        }
        else if (gnss_pst->activePacketBuffer_e == UBLOX_PACKET_PACKETBUF)
        {
            gnss_pst->processUBX(gnss_pst, incoming_u8, &packetBuf_st, requestedClass_u8, requestedID_u8);
        }
        else // if (activePacketBuffer == UBLOX_PACKET_PACKETAUTO)
        {
            gnss_pst->processUBX(gnss_pst, incoming_u8, &packetAuto_st, requestedClass_u8, requestedID_u8);
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

bool Gnss_CheckAutomatic(struct Ublox_Gnss *gnss_pst, uint8_t Class_u8, uint8_t ID_u8)
{
    bool result = false;
    switch (Class_u8)
    {
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

void Gnss_ProcessUBXpacket(struct Ublox_Gnss *gnss_pst, Ubx_Packet_tst *msg_st)
{
    switch (msg_st->cls_u8)
    {
        case UBX_CLASS_CFG:
            if (msg_st->id_u8 == UBX_CFG_RATE && msg_st->len_u16 == UBX_CFG_RATE_LEN)
            {
              //Parse various byte fields into storage - but only if we have memory allocated for it
              if (gnss_pst->packetUBXCFGRATE_pst != NULL)
              {
                  gnss_pst->packetUBXCFGRATE_pst->data.measRate = gnss_pst->extractInt(msg_st, 0);
                  gnss_pst->packetUBXCFGRATE_pst->data.navRate = gnss_pst->extractInt( msg_st, 2);
                  gnss_pst->packetUBXCFGRATE_pst->data.timeRef = gnss_pst->extractInt( msg_st, 4);

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
