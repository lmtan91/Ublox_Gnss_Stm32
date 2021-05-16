#ifndef EXAMPLE_USER_INC_UBLOX_GNSS_H_
#define EXAMPLE_USER_INC_UBLOX_GNSS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "stm32f1xx_hal.h"

#include "dwt_delay.h"
#include <ublox_types.h>
#include <serial.h>

extern uint8_t g_incomming;

extern UBX_ESF_STATUS_t *packetUBXESFSTATUS;
/*******************************************************************************
 * UBX Class IDs.
 ******************************************************************************/
#define UBX_CLASS_NAV 0x01  //Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
#define UBX_CLASS_ACK 0x05    //Ack/Nak Messages: Acknowledge or Reject messages to UBX-CFG input messages
#define UBX_CLASS_CFG 0x06  /*Configuration Input Messages: Configure the receiver.*/
#define UBX_CLASS_MON 0x0A  //Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
#define UBX_CLASS_NMEA  0xF0 //NMEA Strings: standard NMEA strings
#define UBX_CLASS_ESF 0x10    //(NEO-M8P ONLY!!!) External Sensor Fusion Messages: External Sensor Measurements and Status Information

//Class: NAV
//The following are used to configure the NAV UBX messages (navigation results messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 35-36)
#define UBX_NAV_ATT 0x05       //Vehicle "Attitude" Solution
#define UBX_NAV_CLOCK 0x22     //Clock Solution
#define UBX_NAV_DOP 0x04       //Dilution of precision
#define UBX_NAV_EOE 0x61       //End of Epoch
#define UBX_NAV_GEOFENCE 0x39  //Geofencing status. Used to poll the geofence status
#define UBX_NAV_HPPOSECEF 0x13 //High Precision Position Solution in ECEF. Used to find our positional accuracy (high precision).
#define UBX_NAV_HPPOSLLH 0x14  //High Precision Geodetic Position Solution. Used for obtaining lat/long/alt in high precision
#define UBX_NAV_ODO 0x09       //Odometer Solution
#define UBX_NAV_ORB 0x34       //GNSS Orbit Database Info
#define UBX_NAV_POSECEF 0x01   //Position Solution in ECEF
#define UBX_NAV_POSLLH 0x02    //Geodetic Position Solution
#define UBX_NAV_PVT 0x07       //All the things! Position, velocity, time, PDOP, height, h/v accuracies, number of satellites. Navigation Position Velocity Time Solution.
#define UBX_NAV_STATUS 0x03    //Receiver Navigation Status
#define UBX_NAV_VELECEF 0x11 //Velocity Solution in ECEF
#define UBX_NAV_VELNED 0x12  //Velocity Solution in NED
#define UBX_NAV_HPPOSECEF 0x13 //High Precision Position Solution in ECEF. Used to find our positional accuracy (high precision).
#define UBX_NAV_SVIN 0x3B      //Survey-in data. Used for checking Survey In status
#define UBX_NAV_RELPOSNED 0x3C //Relative Positioning Information in NED frame
/*******************************************************************************
 * Class CFG
 ******************************************************************************/
#define UBX_CFG_MSG     0x01       /*Poll a message configuration, or Set Message Rate(s), or Set Message Rate*/
#define UBX_CFG_RATE    0x08      /*Navigation/Measurement Rate Settings. Used to set port baud rates.*/
#define UBX_CFG_PRT     0x00     //Used to configure port specifics. Polls the configuration for one I/O Port, or Port configuration for UART ports, or Port configuration for USB port, or Port configuration for SPI port, or Port configuration for DDC port
#define UBX_CFG_CFG     0x09       //Clear, Save, and Load Configurations. Used to save current configuration
#define UBX_CFG_HNR 	0x5C		//High Navigation Rate
// Class: ACK
#define UBX_ACK_NACK 0x00
#define UBX_ACK_ACK 0x01
#define UBX_ACK_NONE 0x02 //Not a real value

//Class: ESF
// The following constants are used to get External Sensor Measurements and Status
// Information.
#define UBX_ESF_MEAS 0x02
#define UBX_ESF_RAW 0x03
#define UBX_ESF_STATUS 0x10
#define UBX_ESF_ALG 0x14
#define UBX_ESF_INS 0x15 //36 bytes

// Class: NMEA
#define UBX_NMEA_GLL 0x01  //GxGLL (latitude and long, whith time of position fix and status)
#define UBX_NMEA_RMC 0x04  //GxRMC (Recommended minimum data)
#define UBX_NMEA_GGA 0x00  //GxGGA (Global positioning system fix data)
#define UBX_NMEA_VTG 0x05  //GxVTG (course over ground and Ground speed)
#define UBX_NMEA_GSA 0x02  //GxGSA (GNSS DOP and Active satellites)
#define UBX_NMEA_GSV 0x03  //GxGSV (GNSS satellites in view)

//Class: MON
//The following are used to configure the MON UBX messages (monitoring messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 35)
#define UBX_MON_VER 0x04   //Receiver/Software Version. Used for obtaining Protocol Version.

//Registers
#define UBX_SYNCH_1 0xB5
#define UBX_SYNCH_2 0x62


typedef enum commTypes
{
    COMM_TYPE_I2C = 0,
    COMM_TYPE_SERIAL,
    COMM_TYPE_SPI
} Ublox_CommTypes_ten; //Controls which port we look to for incoming bytes

/* Global Status Returns */
typedef enum
{
    UBLOX_STATUS_SUCCESS,
    UBLOX_STATUS_FAIL,
    UBLOX_STATUS_CRC_FAIL,
    UBLOX_STATUS_TIMEOUT,
    UBLOX_STATUS_COMMAND_NACK, // Indicates that the command was unrecognised, invalid or that the module is too busy to respond
    UBLOX_STATUS_OUT_OF_RANGE,
    UBLOX_STATUS_INVALID_ARG,
    UBLOX_STATUS_INVALID_OPERATION,
    UBLOX_STATUS_MEM_ERR,
    UBLOX_STATUS_HW_ERR,
    UBLOX_STATUS_DATA_SENT,     // This indicates that a 'set' was successful
    UBLOX_STATUS_DATA_RECEIVED, // This indicates that a 'get' (poll) was successful
    UBLOX_STATUS_I2C_COMM_FAILURE,
    UBLOX_STATUS_DATA_OVERWRITTEN // This is an error - the data was valid but has been or _is being_ overwritten by another packet
} ublox_status_ten;

/* ubxPacket validity */
typedef enum
{
    UBLOX_PACKET_VALIDITY_NOT_VALID,
    UBLOX_PACKET_VALIDITY_VALID,
    UBLOX_PACKET_VALIDITY_NOT_DEFINED,
    UBLOX_PACKET_NOTACKNOWLEDGED      /*This indicates that we received a NACK*/
} ublox_packet_validity_ten;

// Identify which packet buffer is in use:
// packetCfg (or a custom packet), packetAck or packetBuf
// packetAuto is used to store expected "automatic" messages
typedef enum
{
    UBLOX_PACKET_PACKETCFG,
    UBLOX_PACKET_PACKETACK,
    UBLOX_PACKET_PACKETBUF,
    UBLOX_PACKET_PACKETAUTO
} ublox_packet_buffer_ten;

#ifndef MAX_PAYLOAD_SIZE
// v2.0: keep this for backwards-compatibility, but this is largely superseded by setPacketCfgPayloadSize
#define MAX_PAYLOAD_SIZE 256 //We need ~220 bytes for getProtocolVersion on most ublox modules
//#define MAX_PAYLOAD_SIZE 768 //Worst case: UBX_CFG_VALSET packet with 64 keyIDs each with 64 bit values
#endif

#ifndef defaultMaxWait // Let's allow the user to define their own value if they want to
#define defaultMaxWait 1100
#endif

//The following consts are used to configure the various ports and streams for those ports. See -CFG-PRT.
#define COM_PORT_I2C  0
#define COM_PORT_UART1  1
#define COM_PORT_UART2  2

#define COM_TYPE_UBX (1 << 0)
#define COM_TYPE_NMEA (1 << 1)

typedef enum SentenceTypes
{
    NONE = 0,
    NMEA,
    UBX,
    RTCM
} currentSentence_ten;

typedef struct ubxPacket
{
    uint8_t cls_u8;
    uint8_t id_u8;
    uint16_t len_u16; /*Length of the payload. Does not include cls, id, or checksum bytes*/
    uint16_t counter_u16; /*Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.*/
    uint16_t startingSpot_u16; /*The counter value needed to go past before we begin recording into payload array*/
    uint8_t *payload_pu8; /*We will allocate RAM for the payload if/when needed.*/
    uint8_t checksumA_u8; /*Given to us from module. Checked against the rolling calculated A/B checksums.*/
    uint8_t checksumB_u8;
    ublox_packet_validity_ten valid_en;           /*Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked*/
    ublox_packet_validity_ten classAndIDmatch_en; /*Goes from NOT_DEFINED to VALID or NOT_VALID when the Class and ID match the requestedClass and requestedID*/
} Ubx_Packet_tst;

typedef struct
{
    uint8_t versionLow;      //Loaded from getProtocolVersion().
    uint8_t versionHigh;
    bool moduleQueried;
} moduleSWVersion_t;

typedef struct Ublox_Gnss
{
    // Flag to prevent reentry into checkCallbacks
    // Prevent badness if the user accidentally calls checkCallbacks from inside a callback
    volatile bool checkCallbacksReentrant_b;
    uint16_t rtcmFrameCounter_u16; //Tracks the type of incoming byte inside RTCM frame

    uint16_t ubxFrameCounter_u16;             //It counts all UBX frame. [Fixed header(2bytes), CLS(1byte), ID(1byte), length(2bytes), payload(x bytes), checksums(2bytes)]
    uint8_t rollingChecksumA_u8; //Rolls forward as we receive incoming bytes. Checked against the last two A/B checksum bytes
    uint8_t rollingChecksumB_u8; //Rolls forward as we receive incoming bytes. Checked against the last two A/B checksum bytes

    //Flag if this packet is unrequested (and so should be ignored and not copied into packetCfg or packetAck)
    bool ignoreThisPayload_b;
    //Identify which buffer is in use
    //Data is stored in packetBuf until the requested class and ID can be validated
    //If a match is seen, data is diverted into packetAck or packetCfg
    //"Automatic" messages which have RAM allocated for them are diverted into packetAuto
    ublox_packet_buffer_ten activePacketBuffer_e;
    currentSentence_ten             currentSentence_e;
    UBX_CFG_RATE_tst                *packetUBXCFGRATE_pst;
    Ublox_CommTypes_ten             commTypes_en;
    /* Size for the packetCfg payload. .begin will set this to MAX_PAYLOAD_SIZE if necessary. User can change with setPacketCfgPayloadSize*/
    size_t packetCfgPayloadSize;
    struct Serial              *serial_pst;
}  Ublox_Gnss_tst;

bool Gnss_Init(Ublox_Gnss_tst *gnss_pst, struct Serial *serial_pst);

bool Gnss_ConfMsg(struct Ublox_Gnss *gnss_pst, uint8_t msgClass_u8, uint8_t msgID_u8,
        uint8_t portID_u8, uint8_t sendRate_u8, uint16_t maxWait_u16);

void Gnss_SetPacketCfgPayloadSize(Ublox_Gnss_tst *gnss_pst, size_t payloadSz);
ublox_status_ten Gnss_SendCmd(Ublox_Gnss_tst *gnss_pst, Ubx_Packet_tst *outgoingUBX, uint16_t maxWait, bool expectACKonly);
void Gnss_SendSerialCommand(struct Ublox_Gnss *gnss_pst, Ubx_Packet_tst *outgoingUBX);
void Gnss_CalcChecksum(struct Ublox_Gnss *gnss_pst, Ubx_Packet_tst *msg_pst);
ublox_status_ten Gnss_WaitForACKResponse(struct Ublox_Gnss *gnss_pst, Ubx_Packet_tst *outgoingUBX_pst, uint8_t requestedClass_u8, uint8_t requestedID_u8, uint16_t maxTime_u16);
ublox_status_ten Gnss_WaitForNoACKResponse(struct Ublox_Gnss *gnss_pst, Ubx_Packet_tst *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime);

// Check if any callbacks need to be called
void Gnss_CheckCallbacks(struct Ublox_Gnss *gnss_pst);

bool Gnss_EnableNMEAMessage(struct Ublox_Gnss *gnss_pst, uint8_t msgID_u8, uint8_t portID_u8, uint8_t sendRate_u8, uint16_t maxWait_u16);
bool Gnss_DisableNMEAMessage(struct Ublox_Gnss *gnss_pst, uint8_t msgID_u8, uint8_t portID_u8, uint16_t maxWait_u16);
bool Gnss_IsConnectedT(struct Ublox_Gnss *gnss_pst, uint16_t maxWait_u16);
bool Gnss_IsConnected(struct Ublox_Gnss *gnss_pst);
bool Gnss_GetNavigationFrequencyInternal(struct Ublox_Gnss *gnss_pst, uint16_t maxWait_u16);
bool Gnss_InitPacketUBXCFGRATE(Ublox_Gnss_tst *gnss_pst);

bool Gnss_CheckUblox(struct Ublox_Gnss *gnss_pst, uint8_t requestedClass_u8, uint8_t requestedID_u8); //Checks module with user selected commType
bool Gnss_CheckUbloxInternal(struct Ublox_Gnss *gnss_pst, Ubx_Packet_tst *incomingUBX_pst, uint8_t requestedClass_u8, uint8_t requestedID_u8);
bool Gnss_CheckUbloxSerial(struct Ublox_Gnss *gnss_pst, Ubx_Packet_tst *incomingUBX_pst, uint8_t requestedClass_u8, uint8_t requestedID_u8);
bool Gnss_GetPortSettings(struct Ublox_Gnss *gnss_pst, uint8_t portID_u8, uint16_t maxWait_u16);
bool Gnss_SetPortOutput(struct Ublox_Gnss *gnss_pst, uint8_t portID, uint8_t comSettings, uint16_t maxWait);
bool Gnss_SetUART1Output(struct Ublox_Gnss *gnss_pst, uint8_t comSettings, uint16_t maxWait);
bool Gnss_SetUART2Output(struct Ublox_Gnss *gnss_pst, uint8_t comSettings, uint16_t maxWait);
void Gnss_PrintPacket(struct Ublox_Gnss *gnss_pst, Ubx_Packet_tst *packet_st, bool alwaysPrintPayload_b);
void Gnss_Process(struct Ublox_Gnss *gnss_pst, uint8_t incoming_u8, Ubx_Packet_tst *incomingUBX_pst, uint8_t requestedClass_u8, uint8_t requestedID_u8);
void Gnss_ProcessUBX(struct Ublox_Gnss *gnss_pst, uint8_t incoming_u8, Ubx_Packet_tst *incomingUBX_pst, uint8_t requestedClass_u8, uint8_t requestedID_u8);

void Gnss_SetSerialRate(struct Ublox_Gnss *gnss_pst, uint32_t baudrate_u32, uint8_t uartPort_u8, uint16_t maxTime_y16);
bool Gnss_CheckAutomatic(struct Ublox_Gnss *gnss_pst, uint8_t Class_u8, uint8_t ID_u8);

uint16_t Gnss_GetMaxPayloadSize(struct Ublox_Gnss *gnss_pst, uint8_t Class_u8, uint8_t ID_u8);

void Gnss_ProcessUBXpacket(struct Ublox_Gnss *gnss_pst, Ubx_Packet_tst *msg_st);
uint16_t Gnss_ExtractInt(Ubx_Packet_tst *msg_st, uint8_t spotToStart_u8);
void Gnss_AddToChecksum(struct Ublox_Gnss *gnss_pst, uint8_t incoming_u8);
bool Gnss_SetNavigationFrequency(struct Ublox_Gnss *gnss_pst, uint8_t navFreq_u8, uint16_t maxWait_u16);

bool Gnss_SetAutoPVTcallback(struct Ublox_Gnss *gnss_pst, void (*callbackPointer)(UBX_NAV_PVT_data_t), uint16_t maxWait_u16); //Enable automatic PVT reports at the navigation frequency. Data is accessed from the callback.
bool Gnss_GetPVT(struct Ublox_Gnss *gnss_pst, uint16_t maxWait_u16);  //Query module for latest group of datums and load global vars: lat, long, alt, speed, SIV, accuracies, etc. If autoPVT is disabled, performs an explicit poll and waits, if enabled does not block. Returns true if new PVT is available.
bool Gnss_SetAutoPVTrate(struct Ublox_Gnss *gnss_pst, uint8_t rate_u8, bool implicitUpdate_b, uint16_t maxWait_u16); //Set the rate for automatic PVT reports
bool Gnss_SetAutoPVT(struct Ublox_Gnss *gnss_pst, bool enabled_b, uint16_t maxWait_u16);
bool Gnss_SetAutoPVTImplicit(struct Ublox_Gnss *gnss_pst, bool enabled_b, bool implicitUpdate_b, uint16_t maxWait_u16);

bool Gnss_SetAutoNAVODOrate(struct Ublox_Gnss *gnss_pst,uint8_t rate, bool implicitUpdate_b, uint16_t maxWait_u16); //Set the rate for automatic ODO reports
bool Gnss_SetAutoNAVODO(struct Ublox_Gnss *gnss_pst,bool enabled, uint16_t maxWait_u16);  //Enable/disable automatic ODO reports at the navigation frequency
bool Gnss_SetAutoNAVODOImplicit(struct Ublox_Gnss *gnss_pst,bool enabled, bool implicitUpdate_b, uint16_t maxWait_u16);
bool Gnss_SetAutoNAVODOcallback(struct Ublox_Gnss *gnss_pst, void (*callbackPointer)(UBX_NAV_ODO_data_t), uint16_t maxWait_u16); //Enable automatic ODO reports at the navigation frequency. Data is accessed from the callback.

bool Gnss_GetGnssFixOk(struct Ublox_Gnss *gnss_pst,uint16_t maxWait_u16); //Get whether we have a valid fix (i.e within DOP & accuracy masks)
uint8_t Gnss_GetFixType(struct Ublox_Gnss *gnss_pst,uint16_t maxWait_u16); //Returns the type of fix: 0=no, 3=3D, 4=GNSS+Deadreckoning
uint8_t Gnss_GetSIV(struct Ublox_Gnss *gnss_pst, uint16_t maxWait_u16); //Returns number of sats used in fix
int32_t Gnss_GetLongitude(struct Ublox_Gnss *gnss_pst,uint16_t maxWait_u16); //Returns the current longitude in degrees * 10-7. Auto selects between HighPrecision and Regular depending on ability of module.
int32_t Gnss_GetLatitude(struct Ublox_Gnss *gnss_pst,uint16_t maxWait_u16); //Returns the current latitude in degrees * 10^-7. Auto selects between HighPrecision and Regular depending on ability of module.
int32_t Gnss_GetAltitude(struct Ublox_Gnss *gnss_pst,uint16_t maxWait_u16);
int32_t Gnss_GetAltitudeMSL(struct Ublox_Gnss *gnss_pst,uint16_t maxWait_u16); //Returns the current altitude in mm above mean sea level
int32_t Gnss_GetGroundSpeed(struct Ublox_Gnss *gnss_pst,uint16_t maxWait_u16); //Returns speed in mm/s
uint32_t Gnss_GetSpeedAccEst(struct Ublox_Gnss *gnss_pst,uint16_t maxWait_u16);

// The initPacket functions need to be private as they don't check if memory has already been allocated.
// Functions like setAutoNAVPOSECEF will check that memory has not been allocated before calling initPacket.
bool Gnss_InitPacketUBXNAVPVT(struct Ublox_Gnss *gnss_pst);
bool Gnss_InitModuleSWVersion(); // Allocate RAM for moduleSWVersion and initialize it
bool Gnss_InitPacketUBXNAVODO(struct Ublox_Gnss *gnss_pst);
bool Gnss_InitPacketUBXESFALG(struct Ublox_Gnss *gnss_pst);
bool Gnss_InitPacketUBXESFINS(struct Ublox_Gnss *gnss_pst);
bool Gnss_InitPacketUBXESFMEAS(struct Ublox_Gnss *gnss_pst);
bool Gnss_InitPacketUBXESFSTATUS(struct Ublox_Gnss *gnss_pst);

bool Gnss_SetHNRNavigationRate(struct Ublox_Gnss *gnss_pst, uint8_t rate, uint16_t maxWait);

bool Gnss_SetAutoESFALG(struct Ublox_Gnss *gnss_pst,bool enabled, uint16_t maxWait); //Enable/disable automatic ESF ALG reports
bool Gnss_SetAutoESFALGImplicit(struct Ublox_Gnss *gnss_pst,bool enabled, bool implicitUpdate, uint16_t maxWait); //Enable/disable automatic ESF ALG reports, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
bool Gnss_SetAutoESFALGcallback(struct Ublox_Gnss *gnss_pst,void (*callbackPointer)(UBX_ESF_ALG_data_t), uint16_t maxWait); //Enable automatic ALG reports at the navigation frequency. Data is accessed from the callback.
bool Gnss_SetAutoESFALGrate(struct Ublox_Gnss *gnss_pst,uint8_t rate, bool implicitUpdate, uint16_t maxWait);

bool Gnss_SetAutoESFINS(struct Ublox_Gnss *gnss_pst,bool enabled, uint16_t maxWait); //Enable/disable automatic ESF INS reports
bool Gnss_SetAutoESFINSImplicit(struct Ublox_Gnss *gnss_pst,bool enabled, bool implicitUpdate, uint16_t maxWait); //Enable/disable automatic ESF INS reports, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
bool Gnss_SetAutoESFINSrate(struct Ublox_Gnss *gnss_pst,uint8_t rate, bool implicitUpdate, uint16_t maxWait); //Set the rate for automatic INS reports
bool Gnss_SetAutoESFINScallback(struct Ublox_Gnss *gnss_pst,void (*callbackPointer)(UBX_ESF_INS_data_t), uint16_t maxWait);

bool Gnss_SetAutoESFMEAS(struct Ublox_Gnss *gnss_pst,bool enabled, uint16_t maxWait); //Enable/disable automatic ESF MEAS reports
bool Gnss_SetAutoESFMEASImplicit(struct Ublox_Gnss *gnss_pst,bool enabled, bool implicitUpdate, uint16_t maxWait); //Enable/disable automatic ESF MEAS reports, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
bool Gnss_SetAutoESFMEASrate(struct Ublox_Gnss *gnss_pst,uint8_t rate, bool implicitUpdate, uint16_t maxWait); //Set the rate for automatic MEAS reports
bool Gnss_SetAutoESFMEAScallback(struct Ublox_Gnss *gnss_pst,void (*callbackPointer)(UBX_ESF_MEAS_data_t), uint16_t maxWait); //Enable automatic MEAS reports at the navigation frequency. Data is accessed from the callback.

bool Gnss_SetAutoESFSTATUS(struct Ublox_Gnss *gnss_pst,bool enabled, uint16_t maxWait ); //Enable/disable automatic ESF STATUS reports
bool Gnss_SetAutoESFSTATUSImplicit(struct Ublox_Gnss *gnss_pst,bool enabled, bool implicitUpdate, uint16_t maxWait ); //Enable/disable automatic ESF STATUS reports, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
bool Gnss_SetAutoESFSTATUSrate(struct Ublox_Gnss *gnss_pst,uint8_t rate, bool implicitUpdate , uint16_t maxWait ); //Set the rate for automatic STATUS reports
bool Gnss_SetAutoESFSTATUScallback(struct Ublox_Gnss *gnss_pst,void (*callbackPointer)(UBX_ESF_STATUS_data_t), uint16_t maxWait );

bool Gnss_GetEsfInfo(struct Ublox_Gnss *gnss_pst, uint16_t maxWait_u16); // ESF STATUS Helper
bool Gnss_GetESFSTATUS(struct Ublox_Gnss *gnss_pst, uint16_t maxWait_u16); // ESF STATUS

//Read the module's protocol version
uint8_t Gnss_GetProtocolVersionHigh(struct Ublox_Gnss *gnss_pst,uint16_t maxWait); //Returns the PROTVER XX.00 from UBX-MON-VER register
uint8_t Gnss_GetProtocolVersionLow(struct Ublox_Gnss *gnss_pst,uint16_t maxWait);   //Returns the PROTVER 00.XX from UBX-MON-VER register
bool Gnss_GetProtocolVersion(struct Ublox_Gnss *gnss_pst,uint16_t maxWait);      //Queries module, loads low/high bytes

#ifdef __cplusplus
}
#endif

#endif /* EXAMPLE_USER_INC_UBLOX_GNSS_H_ */
