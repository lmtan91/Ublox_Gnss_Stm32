/* ********************************************************************************************************************
 *
 * COPYRIGHT RESERVED, 2021 VinFast Trading and Production LLC. All rights reserved.
 * The reproduction, distribution and utilization of this document as well as the communication of its contents to
 * others without explicit authorization is prohibited. Offenders will be held liable for the payment of damages.
 * All rights reserved in the event of the grant of a patent, utility model or design.
 *
 ******************************************************************************************************************* */
#ifndef EXAMPLE_USER_INC_UBLOX_TYPES_H_
#define EXAMPLE_USER_INC_UBLOX_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

// UBX-NAV-POSECEF (0x01 0x01): Position solution in ECEF
#define UBX_NAV_POSECEF_LEN 20
typedef struct
{
  uint32_t iTOW; // GPS time of week of the navigation epoch: ms
  int32_t ecefX; // ECEF X coordinate: cm
  int32_t ecefY; // ECEF Y coordinate: cm
  int32_t ecefZ; // ECEF Z coordinate: cm
  uint32_t pAcc; // Position Accuracy Estimate: cm
} UBX_NAV_POSECEF_data_t;

// UBX-NAV-STATUS (0x01 0x03): Receiver navigation status
#define UBX_NAV_STATUS_LEN 16
typedef struct
{
  uint32_t iTOW; // GPS time of week of the navigation epoch: ms
  uint8_t gpsFix; // GPSfix Type: 0x00 = no fix; 0x01 = dead reckoning only; 0x02 = 2D-fix; 0x03 = 3D-fix
                  // 0x04 = GPS + dead reckoning combined; 0x05 = Time only fix; 0x06..0xff = reserved
  union
  {
    uint8_t all;
    struct
    {
      uint8_t gpsFixOk : 1; // 1 = position and velocity valid and within DOP and ACC Masks.
      uint8_t diffSoln : 1; // 1 = differential corrections were applied
      uint8_t wknSet : 1; // 1 = Week Number valid (see Time Validity section for details)
      uint8_t towSet : 1; // 1 = Time of Week valid (see Time Validity section for details)
    } bits;
  } flags;
  union
  {
    uint8_t all;
    struct
    {
      uint8_t diffCorr : 1; // 1 = differential corrections available
      uint8_t carrSolnValid : 1; // 1 = valid carrSoln
      uint8_t reserved : 4;
      uint8_t mapMatching : 2; // map matching status: 00: none
                               // 01: valid but not used, i.e. map matching data was received, but was too old
                               // 10: valid and used, map matching data was applied
                               // 11: valid and used, map matching data was applied.
    } bits;
  } fixStat;
  union
  {
    uint8_t all;
    struct
    {
      uint8_t psmState : 2; // power save mode state
                            // 0: ACQUISITION [or when psm disabled]
                            // 1: TRACKING
                            // 2: POWER OPTIMIZED TRACKING
                            // 3: INACTIVE
      uint8_t reserved1 : 1;
      uint8_t spoofDetState : 2; // Spoofing detection state
                                 // 0: Unknown or deactivated
                                 // 1: No spoofing indicated
                                 // 2: Spoofing indicated
                                 // 3: Multiple spoofing indications
      uint8_t reserved2 : 1;
      uint8_t carrSoln : 2; // Carrier phase range solution status:
                            // 0: no carrier phase range solution
                            // 1: carrier phase range solution with floating ambiguities
                            // 2: carrier phase range solution with fixed ambiguities
    } bits;
  } flags2;
  uint32_t ttff; // Time to first fix (millisecond time tag): ms
  uint32_t msss; // Milliseconds since Startup / Reset: ms
} UBX_NAV_STATUS_data_t;

// UBX-NAV-DOP (0x01 0x04): Dilution of precision
#define UBX_NAV_DOP_LEN 18

typedef struct
{
  uint32_t iTOW; // GPS time of week of the navigation epoch: ms
  uint16_t gDOP; // Geometric DOP: * 0.01
  uint16_t pDOP; // Position DOP: * 0.01
  uint16_t tDOP; // Time DOP: * 0.01
  uint16_t vDOP; // Vertical DOP: * 0.01
  uint16_t hDOP; // Horizontal DOP: * 0.01
  uint16_t nDOP; // Northing DOP: * 0.01
  uint16_t eDOP; // Easting DOP: * 0.01
} UBX_NAV_DOP_data_t;

// UBX-NAV-ATT (0x01 0x05): Attitude solution
#define UBX_NAV_ATT_LEN 32
typedef struct
{
  uint32_t iTOW; // GPS time of week of the navigation epoch: ms
  uint8_t version; // Message version (0x00 for this version)
  uint8_t reserved1[3];
  int32_t roll; // Vehicle roll: Degrees * 1e-5
  int32_t pitch; // Vehicle pitch: Degrees * 1e-5
  int32_t heading; // Vehicle heading: Degrees * 1e-5
  uint32_t accRoll; // Vehicle roll accuracy (if null, roll angle is not available): Degrees * 1e-5
  uint32_t accPitch; // Vehicle pitch accuracy (if null, roll angle is not available): Degrees * 1e-5
  uint32_t accHeading; // Vehicle heading accuracy (if null, roll angle is not available): Degrees * 1e-5
} UBX_NAV_ATT_data_t;

// UBX-NAV-PVT (0x01 0x07): Navigation position velocity time solution
#define UBX_NAV_PVT_LEN 92
typedef struct
{
  uint32_t iTOW; // GPS time of week of the navigation epoch: ms
  uint16_t year; // Year (UTC)
  uint8_t month; // Month, range 1..12 (UTC)
  uint8_t day; // Day of month, range 1..31 (UTC)
  uint8_t hour; // Hour of day, range 0..23 (UTC)
  uint8_t min; // Minute of hour, range 0..59 (UTC)
  uint8_t sec; // Seconds of minute, range 0..60 (UTC)
  union
  {
    uint8_t all;
    struct
    {
      uint8_t validDate : 1; // 1 = valid UTC Date
      uint8_t validTime : 1; // 1 = valid UTC time of day
      uint8_t fullyResolved : 1; // 1 = UTC time of day has been fully resolved (no seconds uncertainty).
      uint8_t validMag : 1; // 1 = valid magnetic declination
    } bits;
  } valid;
  uint32_t tAcc; // Time accuracy estimate (UTC): ns
  int32_t nano; // Fraction of second, range -1e9 .. 1e9 (UTC): ns
  uint8_t fixType; // GNSSfix Type:
                      // 0: no fix
                      // 1: dead reckoning only
                      // 2: 2D-fix
                      // 3: 3D-fix
                      // 4: GNSS + dead reckoning combined
                      // 5: time only fix
  union
  {
    uint8_t all;
    struct
    {
      uint8_t gnssFixOK : 1; // 1 = valid fix (i.e within DOP & accuracy masks)
      uint8_t diffSoln : 1; // 1 = differential corrections were applied
      uint8_t psmState : 3;
      uint8_t headVehValid : 1; // 1 = heading of vehicle is valid, only set if the receiver is in sensor fusion mode
      uint8_t carrSoln : 2; // Carrier phase range solution status:
                              // 0: no carrier phase range solution
                              // 1: carrier phase range solution with floating ambiguities
                              // 2: carrier phase range solution with fixed ambiguities
    } bits;
  } flags;
  union
  {
    uint8_t all;
    struct
    {
      uint8_t reserved : 5;
      uint8_t confirmedAvai : 1; // 1 = information about UTC Date and Time of Day validity confirmation is available
      uint8_t confirmedDate : 1; // 1 = UTC Date validity could be confirmed
      uint8_t confirmedTime : 1; // 1 = UTC Time of Day could be confirmed
    } bits;
  } flags2;
  uint8_t numSV; // Number of satellites used in Nav Solution
  int32_t lon; // Longitude: deg * 1e-7
  int32_t lat; // Latitude: deg * 1e-7
  int32_t height; // Height above ellipsoid: mm
  int32_t hMSL; // Height above mean sea level: mm
  uint32_t hAcc; // Horizontal accuracy estimate: mm
  uint32_t vAcc; // Vertical accuracy estimate: mm
  int32_t velN; // NED north velocity: mm/s
  int32_t velE; // NED east velocity: mm/s
  int32_t velD; // NED down velocity: mm/s
  int32_t gSpeed; // Ground Speed (2-D): mm/s
  int32_t headMot; // Heading of motion (2-D): deg * 1e-5
  uint32_t sAcc; // Speed accuracy estimate: mm/s
  uint32_t headAcc; // Heading accuracy estimate (both motion and vehicle): deg * 1e-5
  uint16_t pDOP; // Position DOP * 0.01
  union
  {
    uint8_t all;
    struct
    {
      uint8_t invalidLlh : 1; // 1 = Invalid lon, lat, height and hMSL
    } bits;
  } flags3;
  uint8_t reserved1[5];
  int32_t headVeh; // Heading of vehicle (2-D): deg * 1e-5
  int16_t magDec; // Magnetic declination: deg * 1e-2
  uint16_t magAcc; // Magnetic declination accuracy: deg * 1e-2
} UBX_NAV_PVT_data_t;

// UBX-NAV-ODO (0x01 0x09): Odometer solution
#define UBX_NAV_ODO_LEN 20
typedef struct
{
  uint8_t version; // Message version (0x00 for this version)
  uint8_t reserved1[3];
  uint32_t iTOW; // GPS time of week of the navigation epoch: ms
  uint32_t distance; // Ground distance since last reset: m
  uint32_t totalDistance; // Total cumulative ground distance: m
  uint32_t distanceStd; // Ground distance accuracy (1-sigma): m
} UBX_NAV_ODO_data_t;

// UBX-NAV-VELECEF (0x01 0x11): Velocity solution in ECEF
#define UBX_NAV_VELECEF_LEN 20
typedef struct
{
  uint32_t iTOW; // GPS time of week of the navigation epoch: ms
  int32_t ecefVX; // ECEF X velocity: cm/s
  int32_t ecefVY; // ECEF Y velocity: cm/s
  int32_t ecefVZ; // ECEF Z velocity: cm/s
  uint32_t sAcc; // Speed accuracy estimate: cm/s
} UBX_NAV_VELECEF_data_t;

// UBX-NAV-VELNED (0x01 0x12): Velocity solution in NED frame
#define UBX_NAV_VELNED_LEN 36
typedef struct
{
  uint32_t iTOW; // GPS time of week of the navigation epoch: ms
  int32_t velN; // North velocity component: cm/s
  int32_t velE; // East velocity component: cm/s
  int32_t velD; // Down velocity component: cm/s
  uint32_t speed; // Speed (3-D): cm/s
  uint32_t gSpeed; // Ground Speed (2-D): cm/s
  int32_t heading; // Heading of motion 2-D: Degrees * 1e-5
  uint32_t sAcc; // Speed accuracy estimate: cm/s
  uint32_t cAcc; // Course/Heading accuracy estimate: Degrees * 1e-5
} UBX_NAV_VELNED_data_t;

// UBX-NAV-HPPOSECEF (0x01 0x13): High precision position solution in ECEF
#define UBX_NAV_HPPOSECEF_LEN 28
typedef struct
{
  uint8_t version; // Message version (0x00 for this version)
  uint8_t reserved1[3];
  uint32_t iTOW; // GPS time of week of the navigation epoch: ms
  int32_t ecefX; // ECEF X coordinate: cm
  int32_t ecefY; // ECEF Y coordinate: cm
  int32_t ecefZ; // ECEF Z coordinate: cm
  int8_t ecefXHp; // High precision component of ECEF X coordinate: mm * 0.1
  int8_t ecefYHp; // High precision component of ECEF Y coordinate: mm * 0.1
  int8_t ecefZHp; // High precision component of ECEF Z coordinate: mm * 0.1
  union
  {
    uint8_t all;
    struct
    {
      uint8_t invalidEcef : 1; // 1 = Invalid ecefX, ecefY, ecefZ, ecefXHp, ecefYHp and ecefZHp
    } bits;
  } flags;
  uint32_t pAcc; // Position Accuracy Estimate: mm * 0.1
} UBX_NAV_HPPOSECEF_data_t;

// UBX-NAV-HPPOSLLH (0x01 0x14): High precision geodetic position solution
#define UBX_NAV_HPPOSLLH_LEN 36
typedef struct
{
  uint8_t version; // Message version (0x00 for this version)
  uint8_t reserved1[2];
  union
  {
    uint8_t all;
    struct
    {
      uint8_t invalidLlh : 1; // 1 = Invalid lon, lat, height, hMSL, lonHp, latHp, heightHp and hMSLHp
    } bits;
  } flags;
  uint32_t iTOW; // GPS time of week of the navigation epoch: ms
  int32_t lon; // Longitude: deg * 1e-7
  int32_t lat; // Latitude: deg * 1e-7
  int32_t height; // Height above ellipsoid: mm
  int32_t hMSL; // Height above mean sea level: mm
  int8_t lonHp; // High precision component of longitude: deg * 1e-9
  int8_t latHp; // High precision component of latitude: deg * 1e-9
  int8_t heightHp; // High precision component of height above ellipsoid: mm * 0.1
  int8_t hMSLHp; // High precision component of height above mean sea level: mm * 0.1
  uint32_t hAcc; // Horizontal accuracy estimate: mm * 0.1
  uint32_t vAcc; // Vertical accuracy estimate: mm * 0.1
} UBX_NAV_HPPOSLLH_data_t;

// UBX-NAV-CLOCK (0x01 0x22): Clock solution
#define UBX_NAV_CLOCK_LEN 20
typedef struct
{
  uint32_t iTOW; // GPS time of week of the navigation epoch: ms
  int32_t clkB; // Clock bias: ns
  int32_t clkD; // Clock drift: ns/s
  uint32_t tAcc; // Time accuracy estimate: ns
  uint32_t fAcc; // Frequency accuracy estimate: ps/s
} UBX_NAV_CLOCK_data_t;

// UBX-NAV-SVIN (0x01 0x3B): Survey-in data
#define UBX_NAV_SVIN_LEN 40
typedef struct
{
  uint8_t version; // Message version (0x00 for this version)
  uint8_t reserved1[3];
  uint32_t iTOW; // GPS time of week of the navigation epoch: ms
  uint32_t dur; // Passed survey-in observation time: s
  int32_t meanX; // Current survey-in mean position ECEF X coordinate: cm
  int32_t meanY; // Current survey-in mean position ECEF Y coordinate: cm
  int32_t meanZ; // Current survey-in mean position ECEF Z coordinate: cm
  int8_t meanXHP; // Current high-precision survey-in mean position ECEF X coordinate: mm * 0.1
  int8_t meanYHP; // Current high-precision survey-in mean position ECEF Y coordinate: mm * 0.1
  int8_t meanZHP; // Current high-precision survey-in mean position ECEF Z coordinate: mm * 0.1
  uint8_t reserved2;
  uint32_t meanAcc; // Current survey-in mean position accuracy: mm * 0.1
  uint32_t obs; // Number of position observations used during survey-in
  int8_t valid; // Survey-in position validity flag, 1 = valid, otherwise 0
  int8_t active; // Survey-in in progress flag, 1 = in-progress, otherwise 0
  uint8_t reserved3[2];
} UBX_NAV_SVIN_data_t;

// UBX-NAV-RELPOSNED (0x01 0x3C): Relative positioning information in NED frame
// Note:
//  RELPOSNED on the M8 is only 40 bytes long
//  RELPOSNED on the F9 is 64 bytes long and contains much more information
#define UBX_NAV_RELPOSNED_LEN 40
#define UBX_NAV_RELPOSNED_LEN_F9 64
typedef struct
{
  uint8_t version; // Message version (0x00 for this version)
  uint8_t reserved0;
  uint16_t refStationId; // Reference Station ID
  uint32_t iTOW; // GPS time of week of the navigation epoch: ms
  int32_t relPosN; // North component of relative position vector: cm
  int32_t relPosE; // East component of relative position vector: cm
  int32_t relPosD; // Down component of relative position vector: cm
  int32_t relPosLength; // Length of the relative position vector: cm
  int32_t relPosHeading; // Heading of the relative position vector: Degrees * 1e-5
  uint8_t reserved1[4];
  int8_t relPosHPN; // High-precision North component of relative position vector: mm * 0.1
  int8_t relPosHPE; // High-precision East component of relative position vector: mm * 0.1
  int8_t relPosHPD; // High-precision Down component of relative position vector: mm * 0.1
  int8_t relPosHPLength; // High-precision component of the length of the relative position vector: mm * 0.1
  uint32_t accN; // Accuracy of relative position North component: mm * 0.1
  uint32_t accE; // Accuracy of relative position East component: mm * 0.1
  uint32_t accD; // Accuracy of relative position Down component: mm * 0.1
  uint32_t accLength; // Accuracy of length of the relative position vector: mm * 0.1
  uint32_t accHeading; // Accuracy of heading of the relative position vector: Degrees * 1e-5
  uint8_t reserved2[4];
  union
  {
    uint32_t all;
    struct
    {
      uint32_t gnssFixOK : 1; // A valid fix (i.e within DOP & accuracy masks)
      uint32_t diffSoln : 1; // 1 if differential corrections were applied
      uint32_t relPosValid : 1; // 1 if relative position components and accuracies are valid
      uint32_t carrSoln : 2; // Carrier phase range solution status:
                              // 0 = no carrier phase range solution
                              // 1 = carrier phase range solution with floating ambiguities
                              // 2 = carrier phase range solution with fixed ambiguities
      uint32_t isMoving : 1; // 1 if the receiver is operating in moving baseline mode
      uint32_t refPosMiss : 1; // 1 if extrapolated reference position was used to compute moving baseline solution this epoch
      uint32_t refObsMiss : 1; // 1 if extrapolated reference observations were used to compute moving baseline solution this epoch
      uint32_t relPosHeadingValid : 1; // 1 if relPosHeading is valid
      uint32_t relPosNormalized : 1; // 1 if the components of the relative position vector (including the high-precision parts) are normalized
    } bits;
  } flags;
} UBX_NAV_RELPOSNED_data_t;

// UBX-CFG-RATE (0x06 0x08): Navigation/measurement rate settings
#define UBX_CFG_RATE_LEN 6
typedef struct
{
  uint16_t measRate; // The elapsed time between GNSS measurements, which defines the rate: ms
  uint16_t navRate; // The ratio between the number of measurements and the number of navigation solutions: cycles
  uint16_t timeRef; // The time system to which measurements are aligned: 0: UTC; 1: GPS; 2: GLONASS; 3: BeiDou; 4: Galileo
} UBX_CFG_RATE_data_t;

typedef struct ubxAutomaticFlags
{
  union
  {
    uint8_t all;
    struct
    {
      uint8_t automatic : 1; // Will this message be delivered and parsed "automatically" (without polling)
      uint8_t implicitUpdate : 1; // Is the update triggered by accessing stale data (=true) or by a call to checkUblox (=false)
      uint8_t addToFileBuffer : 1; // Should the raw UBX data be added to the file buffer?
      uint8_t callbackCopyValid : 1; // Is the copy of the data struct used by the callback valid/fresh?
    } bits;
  } flags;
} ubxAutomaticFlags_tst;

typedef struct
{
  uint16_t measRate; // The elapsed time between GNSS measurements, which defines the rate: ms
  uint16_t navRate; // The ratio between the number of measurements and the number of navigation solutions: cycles
  uint16_t timeRef; // The time system to which measurements are aligned: 0: UTC; 1: GPS; 2: GLONASS; 3: BeiDou; 4: Galileo
} UBX_CFG_RATE_data_tst;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t measRate : 1;
      uint32_t navRate : 1;
      uint32_t timeRef : 1;
    } bits;
  } moduleQueried;
} UBX_CFG_RATE_moduleQueried_tst;

typedef struct
{
    ubxAutomaticFlags_tst automaticFlags;
    UBX_CFG_RATE_data_tst data;
    UBX_CFG_RATE_moduleQueried_tst moduleQueried;
    void (*callbackPointer)(UBX_CFG_RATE_data_tst);
    UBX_CFG_RATE_data_tst  *callbackData;
} UBX_CFG_RATE_tst;

#ifdef __cplusplus
}
#endif

#endif /* EXAMPLE_USER_INC_UBLOX_TYPES_H_ */
